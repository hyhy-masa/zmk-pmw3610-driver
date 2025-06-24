/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

// 12-bit two's complement value to int16_t
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "pmw3610.h"
#include "pixart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

//////// Sensor initialization steps definition ////////
enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

/* Timings (in ms) needed between steps */
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 10,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
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

/* Forward declarations */
static void pmw3610_async_init(struct k_work *work);
static int pmw3610_init_irq(const struct device *dev);
static int pmw3610_set_cpi(const struct device *dev, uint16_t cpi);
static int pmw3610_read_burst(const struct device *dev, uint8_t *buf);

/**
 * Control the chip-select (CS) line for the sensor.
 */
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (enable) {
#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
        err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
        if (err) {
            LOG_ERR("Failed to configure CS GPIO as output");
        }
#endif
        err = gpio_pin_set_dt(&config->cs_gpio, 1);
        if (err) {
            LOG_ERR("SPI CS assert failed");
        }
        k_busy_wait(T_NCS_SCLK);
    } else {
        k_busy_wait(T_NCS_SCLK);
        err = gpio_pin_set_dt(&config->cs_gpio, 0);
        if (err) {
            LOG_ERR("SPI CS release failed");
        }
#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
        int cfgerr = gpio_pin_configure_dt(&config->cs_gpio, GPIO_INPUT);
        if (cfgerr) {
            LOG_ERR("Failed to set CS GPIO to high impedance");
        }
#endif
    }

    return err;
}

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    
    // Disable interrupt during processing
    const struct pixart_config *config = data->dev->config;
    gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_DISABLE);

    // Submit the work
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    uint8_t burst_buf[PMW3610_BURST_SIZE];
    int16_t x = 0, y = 0;

    // Read motion data
    if (pmw3610_read_burst(dev, burst_buf) == 0) {
        uint8_t motion = burst_buf[0];
        
        if (motion & 0x80) {  // Motion bit set
            // Combine high and low bytes for X and Y
            x = (int16_t)((burst_buf[PMW3610_XY_H_POS] & 0x0F) << 8) | burst_buf[PMW3610_X_L_POS];
            y = (int16_t)((burst_buf[PMW3610_XY_H_POS] & 0xF0) << 4) | burst_buf[PMW3610_Y_L_POS];
            
            // Convert to signed 12-bit values
            x = TOINT16(x, 12);
            y = TOINT16(y, 12);
            
            // Apply CPI divisor and transformations
            x = x / CONFIG_PMW3610_CPI_DIVIDOR;
            y = y / CONFIG_PMW3610_CPI_DIVIDOR;
            
            // Apply orientation
#if defined(CONFIG_PMW3610_ORIENTATION_90)
            int16_t temp = x;
            x = y;
            y = -temp;
#elif defined(CONFIG_PMW3610_ORIENTATION_180)
            x = -x;
            y = -y;
#elif defined(CONFIG_PMW3610_ORIENTATION_270)
            int16_t temp = x;
            x = -y;
            y = temp;
#endif

            // Apply inversion
#ifdef CONFIG_PMW3610_INVERT_X
            x = -x;
#endif
#ifdef CONFIG_PMW3610_INVERT_Y
            y = -y;
#endif

            // Report input event
            if (x != 0) {
                input_report_rel(dev, INPUT_REL_X, x, true, K_FOREVER);
            }
            if (y != 0) {
                input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
            }
            if (x != 0 || y != 0) {
                input_sync(dev);
            }
        }
    }

    // Re-enable interrupt
    const struct pixart_config *config = dev->config;
    gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf = { .buf = &reg, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read: SPI write phase failed");
        spi_cs_ctrl(dev, false);
        return err;
    }

    k_busy_wait(T_SRAD);

    struct spi_buf rx_buf = { .buf = buf, .len = 1 };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read: SPI read phase failed");
        spi_cs_ctrl(dev, false);
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    return 0;
}

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
        spi_cs_ctrl(dev, false);
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    return 0;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;

    err = _reg_write(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    if (err != 0) {
        return err;
    }

    err = _reg_write(dev, reg, val);
    if (err) {
        return err;
    }

    return 0;
}

static int pmw3610_read_burst(const struct device *dev, uint8_t *buf) {
    int err;
    const struct pixart_config *config = dev->config;

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t cmd = PMW3610_REG_MOTION_BURST;
    const struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst: SPI write failed");
        spi_cs_ctrl(dev, false);
        return err;
    }

    k_busy_wait(T_SRAD_MOTBR);

    struct spi_buf rx_buf = { .buf = buf, .len = PMW3610_BURST_SIZE };
    const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst: SPI read failed");
        spi_cs_ctrl(dev, false);
        return err;
    }

    return spi_cs_ctrl(dev, false);
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pixart_data *data = CONTAINER_OF(dwork, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_DBG("Async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("Async init failed at step %d", data->async_init_step);
        return;
    }

    data->async_init_step++;

    if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
        data->ready = true;
        LOG_INF("Init complete");
        // Enable interrupt
        gpio_pin_interrupt_configure_dt(&((const struct pixart_config *)dev->config)->irq_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE);
    } else {
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
    }
}

static int pmw3610_async_init_power_up(const struct device *dev) {
    return reg_write(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    uint8_t val;
    return reg_read(dev, PMW3610_REG_OBSERVATION, &val);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t val;
    int err = reg_read(dev, PMW3610_REG_OBSERVATION, &val);
    if (err) {
        return err;
    }

    if ((val & 0x0F) != 0x0F) {
        LOG_ERR("Observation register check failed: 0x%02x", val);
        return -EIO;
    }

    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err;

    // Set performance
    err = reg_write(dev, PMW3610_REG_PERFORMANCE, PMW3610_PERFORMANCE_VALUE);
    if (err) {
        return err;
    }

    // Set CPI
    err = pmw3610_set_cpi(dev, CONFIG_PMW3610_CPI);
    if (err) {
        return err;
    }

    // Configure run downshift time
    uint8_t run_downshift = CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS / 32;
    err = reg_write(dev, PMW3610_REG_RUN_DOWNSHIFT, run_downshift);
    if (err) {
        return err;
    }

    // Configure rest periods
    err = reg_write(dev, PMW3610_REG_REST1_PERIOD, CONFIG_PMW3610_REST1_SAMPLE_TIME_MS / 10);
    if (err) {
        return err;
    }

    // Clear motion registers
    uint8_t dummy;
    reg_read(dev, PMW3610_REG_DELTA_X_L, &dummy);
    reg_read(dev, PMW3610_REG_DELTA_Y_L, &dummy);
    reg_read(dev, PMW3610_REG_DELTA_XY_H, &dummy);
    reg_read(dev, PMW3610_REG_MOTION, &dummy);

    return 0;
}

static int pmw3610_set_cpi(const struct device *dev, uint16_t cpi) {
    if (cpi < PMW3610_MIN_CPI || cpi > PMW3610_MAX_CPI || cpi % 200 != 0) {
        LOG_ERR("Invalid CPI: %d", cpi);
        return -EINVAL;
    }

    uint8_t res_step = (cpi / 200) - 1;
    return reg_write(dev, PMW3610_REG_RES_STEP, res_step);
}

static int pmw3610_init_irq(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
        return err;
    }

    return 0;
}

static int pmw3610_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    data->dev = dev;
    data->sw_smart_flag = false;

    k_work_init(&data->trigger_work, pmw3610_work_callback);

    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#define PMW3610_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);                    \
    static int32_t snipe_layers##n[] = DT_PROP(DT_DRV_INST(n), snipe_layers);                      \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
