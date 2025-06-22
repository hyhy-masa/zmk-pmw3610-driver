/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

//////// Sensor initialization steps definition ////////
// init is done in non-blocking manner (i.e., async), a //
// delayable work is defined for this purpose           //
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,  // reset cs line and assert power-up reset
    ASYNC_INIT_STEP_CLEAR_OB1, // clear observation1 register for self-test check
    ASYNC_INIT_STEP_CHECK_OB1, // check the value of observation1 register after self-test check
    ASYNC_INIT_STEP_CONFIGURE, // set other registers like CPI and downshift times (run, rest1, rest2),
                               // and clear motion registers

    ASYNC_INIT_STEP_COUNT // end flag
};

/* Timings (in ms) needed between steps to allow each step to finish successfully. */
// - Since MCU is not involved in the sensor init process, it is allowed to do other tasks.
//   Thus, k_sleep or delayed scheduling can be used for waiting.
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10,  // Datasheet: >5ms required (using 10ms)
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200, // Datasheet: 150 µs required; using 200ms to ensure stable startup
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,  // Datasheet: 10ms required; using 50ms for reliability
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP]   = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1]  = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1]  = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE]  = pmw3610_async_init_configure,
};

//////// Function definitions ////////

/**
 * Control the chip-select (CS) line for the sensor.
 *
 * @param enable True to assert CS (start communication), False to deassert CS.
 */
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (enable) {
#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
        /* Reconfigure CS pin as output (inactive-high) before asserting */
        err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("Failed to configure CS GPIO as output");
            // Continue even if this fails, we will attempt to set CS low
        }
#endif
        /* Drive CS active (pull low, since GPIO is active-low) */
        err = gpio_pin_set_dt(&config->cs_gpio, 1);
        if (err) {
            LOG_ERR("SPI CS assert failed");
        }
        /* Wait T_NCS_SCLK after asserting CS (datasheet timing requirement) */
        k_busy_wait(T_NCS_SCLK);
    } else {
        /* Wait T_NCS_SCLK before releasing CS (ensure clock inactive before deassert) */
        k_busy_wait(T_NCS_SCLK);
        /* Deassert CS (drive it inactive-high) */
        err = gpio_pin_set_dt(&config->cs_gpio, 0);
        if (err) {
            LOG_ERR("SPI CS release failed");
        }
#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
        /* Configure CS pin as high-impedance input after deasserting, to free the line for other uses */
        int cfgerr = gpio_pin_configure_dt(&config->cs_gpio, GPIO_INPUT);
        if (cfgerr) {
            LOG_ERR("Failed to set CS GPIO to high impedance");
        }
#endif
        /* Note: T_SRX post-release delay will be handled after this function returns (in calling code) */
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address (tell sensor which register to read). */
    const struct spi_buf tx_buf = { .buf = &reg, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read: SPI write phase failed");
        (void)spi_cs_ctrl(dev, false);
        return err;
    }

    /* Wait T_SRAD before reading (per datasheet) */
    k_busy_wait(T_SRAD);

    /* Read register value into provided buffer. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read: SPI read phase failed");
        (void)spi_cs_ctrl(dev, false);
        return err;
    }

    /* Deassert CS to end the transaction */
    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    /* Wait T_SRX after releasing CS (post-read delay) */
    k_busy_wait(T_SRX);

    return 0;
}

// Primitive register write (internal helper, does one SPI transaction without managing sensor clock gating).
static int _reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = { (uint8_t)(SPI_WRITE_BIT | reg), val };
    const struct spi_buf tx_buf = { .buf = buf, .len = ARRAY_SIZE(buf) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write: SPI transfer failed");
        (void)spi_cs_ctrl(dev, false);
        return err;
    }

    /* Wait T_SCLK_NCS_WR before deasserting CS (write timing) */
    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    /* Wait T_SWX after write transaction (post-write delay) */
    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    /* Enable sensor SPI clock (required before writing configuration) */
    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (unlikely(err != 0)) {
        return err;
    }

    /* Write the target register */
    err = _reg_write(dev, reg, val);
    if (err) {
        return err;
    }

    return 0;
}

// ... (rest of file remains unchanged, including initialization and data processing logic) ...
