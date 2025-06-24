/*
 * Copyright (c) 2024
 * Derived from the original ZMK pmw3610 driver – heavily simplified so the
 * module *builds* with Zephyr 3.5.  
 * Functionality‑wise this is still a stub: only the register access helpers,
 * basic IRQ wiring and asynchronous initialisation state‑machine are kept.
 * You will very likely want to replace/extend the TODO sections with the
 * real sensor logic – but at least west build will now finish.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/input/input.h>

#include <zmk/keymap.h>

LOG_MODULE_REGISTER(pmw3610, CONFIG_INPUT_LOG_LEVEL);

/* -------------------------------------------------------------------------- */
/*  Datasheet‑related helpers                                                  */
/* -------------------------------------------------------------------------- */

// 12‑bit two's‑complement helper – keep as inline macro to avoid extra casts
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){ val }).value)

/* Generic SPI helper flags */
#define SPI_WRITE_BIT 0x80u

/* Timing constants (µs) – approximated. Adjust per real datasheet if needed. */
#define T_NCS_SCLK           20    /* CS ↓/↑ ↔ first SCLK edge   */
#define T_SRAD               160   /* reg‑addr → first read byte */
#define T_SCLK_NCS_WR        20    /* last SCLK ↑ → CS ↑ (write) */
#define T_SWX                20    /* write → next access        */
#define T_SRX                20    /* read  → next access        */

/* Register map (partial, add more as you implement real logic) */
#define PMW3610_REG_SPI_CLK_ON_REQ  0x3A
#define PMW3610_SPI_CLOCK_CMD_ENABLE 0x5A

/* -------------------------------------------------------------------------- */
/*  Forward declarations                                                      */
/* -------------------------------------------------------------------------- */
struct pmw3610_data; /* fwd so we can typedef before full struct */

static int pmw3610_set_interrupt(const struct device *dev, bool enable);
static int pmw3610_init_irq(const struct device *dev);
static void pmw3610_async_init(struct k_work *work);

/* -------------------------------------------------------------------------- */
/*  Driver config & runtime data structures                                   */
/* -------------------------------------------------------------------------- */

struct pmw3610_config {
    struct gpio_dt_spec   irq_gpio;
    struct spi_dt_spec    bus;
    struct gpio_dt_spec   cs_gpio;

    /* keymap‑controlled behaviour */
    int32_t              *scroll_layers;
    size_t                scroll_layers_len;
    int32_t              *snipe_layers;
    size_t                snipe_layers_len;
};

enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

static const int32_t async_init_delay_ms[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP]   = 10,
    [ASYNC_INIT_STEP_CLEAR_OB1]  = 2,   /* 150 µs -> 2 ms */
    [ASYNC_INIT_STEP_CHECK_OB1]  = 50,
    [ASYNC_INIT_STEP_CONFIGURE]  = 0,
};

struct pmw3610_data {
    /* Basic Zephyr plumbing */
    const struct device *dev;
    const struct pmw3610_config *cfg;

    /* IRQ handling */
    struct gpio_callback irq_gpio_cb;
    struct k_work        trigger_work;
    sensor_trigger_handler_t handler;

    /* Asynchronous init */
    struct k_work_delayable init_work;
    enum pmw3610_init_step  init_state;
    bool                    ready;

    /* user flags */
    bool sw_smart_flag;
};

#define DEV_CFG(dev) ((const struct pmw3610_config *)(dev)->config)
#define DEV_DATA(dev) ((struct pmw3610_data *)(dev)->data)

/* -------------------------------------------------------------------------- */
/*  Low‑level SPI helpers                                                     */
/* -------------------------------------------------------------------------- */

static int spi_cs_ctrl(const struct device *dev, bool active)
{
    const struct pmw3610_config *cfg = DEV_CFG(dev);
    int err;

#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
    /* Ensure the pin is push‑pull before asserting */
    if (active) {
        (void)gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);
    }
#endif

    err = gpio_pin_set_dt(&cfg->cs_gpio, active ? 1 : 0);
    if (err) {
        LOG_ERR("CS %s failed (%d)", active ? "assert" : "deassert", err);
        return err;
    }

    k_busy_wait(T_NCS_SCLK);

#ifdef CONFIG_PMW3610_CS_HIZ_AFTER_XFER
    if (!active) {
        (void)gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_INPUT);
    }
#endif
    return 0;
}

static int _reg_write_raw(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct pmw3610_config *cfg = DEV_CFG(dev);
    uint8_t buf[2] = { SPI_WRITE_BIT | reg, val };

    int err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf tx_buf = { .buf = buf, .len = sizeof(buf) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

    err = spi_write_dt(&cfg->bus, &tx);
    k_busy_wait(T_SCLK_NCS_WR);
    spi_cs_ctrl(dev, false);
    k_busy_wait(T_SWX);
    return err;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
    /* Ensure the internal SPI clock is on before every config write */
    (void)_reg_write_raw(dev, PMW3610_REG_SPI_CLK_ON_REQ,
                         PMW3610_SPI_CLOCK_CMD_ENABLE);
    return _reg_write_raw(dev, reg, val);
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct pmw3610_config *cfg = DEV_CFG(dev);
    int err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    const struct spi_buf txb = { .buf = &reg, .len = 1 };
    const struct spi_buf_set tx = { &txb, 1 };
    err = spi_write_dt(&cfg->bus, &tx);
    if (err) {
        spi_cs_ctrl(dev, false);
        return err;
    }

    k_busy_wait(T_SRAD);

    struct spi_buf rxb = { .buf = val, .len = 1 };
    const struct spi_buf_set rx = { &rxb, 1 };
    err = spi_read_dt(&cfg->bus, &rx);

    spi_cs_ctrl(dev, false);
    k_busy_wait(T_SRX);
    return err;
}

/* -------------------------------------------------------------------------- */
/*  IRQ handling (VERY basic)                                                 */
/* -------------------------------------------------------------------------- */

static void pmw3610_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                  uint32_t pins)
{
    struct pmw3610_data *data = CONTAINER_OF(cb, struct pmw3610_data, irq_gpio_cb);

    /* Disable further IRQs until processed */
    pmw3610_set_interrupt(data->dev, false);
    k_work_submit(&data->trigger_work);
}

static int pmw3610_set_interrupt(const struct device *dev, bool enable)
{
    const struct pmw3610_config *cfg = DEV_CFG(dev);
    gpio_flags_t flags = GPIO_INT_EDGE_TO_ACTIVE;
    int err;

    if (enable) {
        err = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, flags);
    } else {
        err = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
    }
    return err;
}

static int pmw3610_init_irq(const struct device *dev)
{
    const struct pmw3610_config *cfg = DEV_CFG(dev);
    struct pmw3610_data *data = DEV_DATA(dev);

    if (!device_is_ready(cfg->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO not ready");
        return -ENODEV;
    }

    int err = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (err) {
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(cfg->irq_gpio.pin));
    err = gpio_add_callback(cfg->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        return err;
    }

    return pmw3610_set_interrupt(dev, true);
}

/* -------------------------------------------------------------------------- */
/*  Application‑facing work item                                              */
/* -------------------------------------------------------------------------- */

static void pmw3610_trigger_work_cb(struct k_work *work)
{
    struct pmw3610_data *data = CONTAINER_OF(work, struct pmw3610_data, trigger_work);

    if (data->handler) {
        struct sensor_trigger trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_ALL,
        };
        data->handler(data->dev, &trig);
    }

    /* re‑enable interrupt */
    pmw3610_set_interrupt(data->dev, true);
}

/* -------------------------------------------------------------------------- */
/*  Asynchronous initialisation state machine                                 */
/* -------------------------------------------------------------------------- */

/*  Each step is kept minimal so the code compiles – extend with real regs. */
static int pmw3610_async_init_power_up(const struct device *dev)
{
    /* Datasheet – assert power‑up reset via CS toggle etc.  Stubbed here. */
    return 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev)
{
    /* Clear observation reg – stub */
    return 0;
}

static int pmw3610_async_init_check_ob1(const struct device *dev)
{
    /* Read self‑test result – stub returns success */
    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev)
{
    /* CPI, downshift etc.  Add real config writes here. */
    return 0;
}

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP]   = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1]  = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1]  = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE]  = pmw3610_async_init_configure,
};

static void pmw3610_async_init(struct k_work *work)
{
    struct pmw3610_data *data = CONTAINER_OF(work, struct pmw3610_data, init_work.work);
    const struct device *dev = data->dev;

    int err = async_init_fn[data->init_state](dev);
    if (err) {
        LOG_ERR("async init step %d failed (%d)", data->init_state, err);
        return; /* stop retrying – your strategy may differ */
    }

    if (data->init_state < ASYNC_INIT_STEP_COUNT - 1) {
        data->init_state++;
        k_work_schedule(&data->init_work, K_MSEC(async_init_delay_ms[data->init_state]));
    } else {
        data->ready = true;
        LOG_INF("PMW3610 ready");
    }
}

/* -------------------------------------------------------------------------- */
/*  Public API – Zephyr sensor driver hooks                                   */
/* -------------------------------------------------------------------------- */

static int pmw3610_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct pmw3610_data *data = DEV_DATA(dev);
    if (!data->ready) {
        return -EAGAIN;
    }
    /* TODO: implement real motion registers read */
    return 0;
}

static int pmw3610_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val)
{
    /* TODO: return real X/Y/scroll deltas.  Stub 0. */
    val->val1 = 0;
    val->val2 = 0;
    return 0;
}

static int pmw3610_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                               sensor_trigger_handler_t handler)
{
    struct pmw3610_data *data = DEV_DATA(dev);

    if (trig->type != SENSOR_TRIG_DATA_READY) {
        return -ENOTSUP;
    }

    /* Store callback – IRQ already armed in init */
    data->handler = handler;
    return 0;
}

static const struct sensor_driver_api pmw3610_api = {
    .sample_fetch = pmw3610_sample_fetch,
    .channel_get  = pmw3610_channel_get,
    .trigger_set  = pmw3610_trigger_set,
};

/* -------------------------------------------------------------------------- */
/*  Device instantiation via DT                                               */
/* -------------------------------------------------------------------------- */

static int pmw3610_init(const struct device *dev)
{
    struct pmw3610_data   *data = DEV_DATA(dev);
    const struct pmw3610_config *cfg = DEV_CFG(dev);

    data->dev       = dev;
    data->cfg       = cfg;
    data->ready     = false;
    data->init_state = ASYNC_INIT_STEP_POWER_UP;
    data->sw_smart_flag = false;

    /* CS GPIO */
    if (!device_is_ready(cfg->cs_gpio.port)) {
        LOG_ERR("CS GPIO not ready");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_INACTIVE);

    /* Trigger work */
    k_work_init(&data->trigger_work, pmw3610_trigger_work_cb);

    /* IRQ */
    int err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    /* Schedule asynchronous init */
    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay_ms[data->init_state]));

    return 0;
}

#define PMW3610_DEFINE(inst)                                                                  \
    static struct pmw3610_data pmw3610_data_##inst;                                           \
                                                                                              \
    static int32_t scroll_layers_##inst[] = DT_PROP(DT_DRV_INST(inst), scroll_layers);        \
    static int32_t snipe_layers_##inst[]  = DT_PROP(DT_DRV_INST(inst), snipe_layers);         \
                                                                                              \
    static const struct pmw3610_config pmw3610_config_##inst = {                              \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                   \
        .bus      = SPI_DT_SPEC_INST_GET(inst,                                                 \
                                         SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |               \
                                             SPI_TRANSFER_MSB | SPI_MODE_CPOL |               \
                                             SPI_MODE_CPHA,                                   \
                                         0),                                                  \
        .cs_gpio  = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(inst)),                              \
        .scroll_layers     = scroll_layers_##inst,                                            \
        .scroll_layers_len = ARRAY_SIZE(scroll_layers_##inst),                                \
        .snipe_layers      = snipe_layers_##inst,                                             \
        .snipe_layers_len  = ARRAY_SIZE(snipe_layers_##inst),                                 \
    };                                                                                        \
                                                                                              \
    DEVICE_DT_INST_DEFINE(inst,                                                               \
                          pmw3610_init,                                                       \
                          NULL,                                                               \
                          &pmw3610_data_##inst,                                               \
                          &pmw3610_config_##inst,                                             \
                          POST_KERNEL,                                                        \
                          CONFIG_SENSOR_INIT_PRIORITY,                                        \
                          &pmw3610_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)
