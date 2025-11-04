/*
 * Copyright 2024 Google LLC
 * Modifications Copyright 2025 sekigon-gonnoc
 *
 * Original source code:
 * https://github.com/zephyrproject-rtos/zephyr/blob/19c6240b6865bcb28e1d786d4dcadfb3a02067a0/drivers/input/input_paw32xx.c
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/util.h>

#include "../include/paw3222.h"

LOG_MODULE_REGISTER(paw32xx, CONFIG_ZMK_LOG_LEVEL);

/* forward declaration so DEVICE_DT_INST_DEFINE can reference it */
static int paw32xx_init(const struct device *dev);

#define DT_DRV_COMPAT pixart_paw3222

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define PAW32XX_PRODUCT_ID1 0x00
#define PAW32XX_PRODUCT_ID2 0x01
#define PAW32XX_MOTION 0x02
#define PAW32XX_DELTA_X 0x03
#define PAW32XX_DELTA_Y 0x04
#define PAW32XX_OPERATION_MODE 0x05
#define PAW32XX_CONFIGURATION 0x06
#define PAW32XX_WRITE_PROTECT 0x09
#define PAW32XX_SLEEP1 0x0a
#define PAW32XX_SLEEP2 0x0b
#define PAW32XX_SLEEP3 0x0c
#define PAW32XX_CPI_X 0x0d
#define PAW32XX_CPI_Y 0x0e
#define PAW32XX_DELTA_XY_HI 0x12
#define PAW32XX_MOUSE_OPTION 0x19

#define PRODUCT_ID_PAW32XX 0x30
#define SPI_WRITE BIT(7)

#define MOTION_STATUS_MOTION BIT(7)
#define OPERATION_MODE_SLP_ENH BIT(4)
#define OPERATION_MODE_SLP2_ENH BIT(3)
#define OPERATION_MODE_SLP_MASK (OPERATION_MODE_SLP_ENH | OPERATION_MODE_SLP2_ENH)
#define CONFIGURATION_PD_ENH BIT(3)
#define CONFIGURATION_RESET BIT(7)
#define WRITE_PROTECT_ENABLE 0x00
#define WRITE_PROTECT_DISABLE 0x5a
#define MOUSE_OPTION_MOVX_INV_BIT 3
#define MOUSE_OPTION_MOVY_INV_BIT 4

#define PAW32XX_DATA_SIZE_BITS 8

#define RESET_DELAY_MS 2

#define RES_STEP 38
#define RES_MIN (16 * RES_STEP)
#define RES_MAX (127 * RES_STEP)

struct paw32xx_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec power_gpio;
    int16_t res_cpi;
    bool force_awake;
};

/* Note: struct paw32xx_data is defined in include/paw3222.h when
 * CONFIG_PAW3222_SCROLL_ACCEL is enabled. If not enabled, a minimal
 * opaque data pointer is used via device->data.
 */

static inline int32_t sign_extend(uint32_t value, uint8_t index) {
    __ASSERT_NO_MSG(index <= 31);

    uint8_t shift = 31 - index;

    return (int32_t)(value << shift) >> shift;
}

static int paw32xx_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    const struct spi_buf tx_buf = {
        .buf = &addr,
        .len = sizeof(addr),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf[] = {
        {
            .buf = NULL,
            .len = sizeof(addr),
        },
        {
            .buf = value,
            .len = 1,
        },
    };
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = ARRAY_SIZE(rx_buf),
    };

    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

    return ret;
}

static int paw32xx_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct paw32xx_config *cfg = dev->config;

    uint8_t write_buf[] = { addr | SPI_WRITE, value };
    const struct spi_buf tx_buf = {
        .buf = write_buf,
        .len = sizeof(write_buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write_dt(&cfg->spi, &tx);
}

static int paw32xx_update_reg(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t value) {
    uint8_t val;
    int ret;

    ret = paw32xx_read_reg(dev, addr, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~mask) | (value & mask);

    ret = paw32xx_write_reg(dev, addr, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int paw32xx_read_xy(const struct device *dev, int16_t *x, int16_t *y) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    uint8_t tx_data[] = {
        PAW32XX_DELTA_X,
        0xff,
        PAW32XX_DELTA_Y,
        0xff,
    };
    uint8_t rx_data[sizeof(tx_data)];

    const struct spi_buf tx_buf = {
        .buf = tx_data,
        .len = sizeof(tx_data),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    struct spi_buf rx_buf = {
        .buf = rx_data,
        .len = sizeof(rx_data),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    if (ret < 0) {
        return ret;
    }

    *x = rx_data[1];
    *y = rx_data[3];

    *x = sign_extend(*x, PAW32XX_DATA_SIZE_BITS - 1);
    *y = sign_extend(*y, PAW32XX_DATA_SIZE_BITS - 1);

    return 0;
}

static void paw32xx_motion_timer_handler(struct k_timer *timer) {
    struct paw32xx_data *data = CONTAINER_OF(timer, struct paw32xx_data, motion_timer);
    k_work_submit(&data->motion_work);
}

static void paw32xx_motion_work_handler(struct k_work *work) {
    LOG_DBG("paw32xx_motion_work_handler running");
    struct paw32xx_data *data = CONTAINER_OF(work, struct paw32xx_data, motion_work);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int16_t x, y;
    int ret;

    ret = paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    if (ret < 0) {
        return;
    }

    if ((val & MOTION_STATUS_MOTION) == 0x00) {
        /* No motion detected, re-enable interrupts and wait for next interrupt */
        gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);

        if (gpio_pin_get_dt(&cfg->irq_gpio) == 0) {
            return;
        }
    }

    ret = paw32xx_read_xy(dev, &x, &y);
    if (ret < 0) {
        return;
    }

    LOG_DBG("x=%4d y=%4d", x, y);

#ifdef CONFIG_PAW3222_SCROLL_ACCEL
    int32_t accel_x = 0, accel_y = 0;
    paw3222_apply_scroll_accel(x, y, data, &accel_x, &accel_y);

    data->scroll_delta_x += accel_x;
    data->scroll_delta_y += accel_y;

    /* Vertical first (wheel), then horizontal (hwheel) */
    paw3222_process_scroll_events(data->dev, data, data->scroll_delta_y, false);
    paw3222_process_scroll_events(data->dev, data, data->scroll_delta_x, true);
#else
    input_report_rel(data->dev, INPUT_REL_X, x, false, K_FOREVER);
    input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);
#endif

    /* Schedule next check after 15ms without using interrupts */
    k_timer_start(&data->motion_timer, K_MSEC(15), K_NO_WAIT);
}

static void paw32xx_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb,
                                   uint32_t pins) {
    LOG_DBG("paw32xx_motion_handler irq, pins=%u", pins);
    struct paw32xx_data *data = CONTAINER_OF(cb, struct paw32xx_data, motion_cb);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;

    /* Disable interrupts while timer is active */
    gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);

    /* Cancel any pending timer */
    k_timer_stop(&data->motion_timer);

    /* Process motion */
    k_work_submit(&data->motion_work);
}

int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi) {
    uint8_t val;
    int ret;

    if (!IN_RANGE(res_cpi, RES_MIN, RES_MAX)) {
        LOG_ERR("res_cpi out of range: %d", res_cpi);
        return -EINVAL;
    }

    val = res_cpi / RES_STEP;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_X, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_CPI_Y, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

int paw32xx_force_awake(const struct device *dev, bool enable) {
    uint8_t val = enable ? 0 : OPERATION_MODE_SLP_MASK;
    int ret;

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_DISABLE);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_update_reg(dev, PAW32XX_OPERATION_MODE, OPERATION_MODE_SLP_MASK, val);
    if (ret < 0) {
        return ret;
    }

    ret = paw32xx_write_reg(dev, PAW32XX_WRITE_PROTECT, WRITE_PROTECT_ENABLE);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int paw32xx_configure(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int ret;
    int retry_count = 10;

    /* Check if the device is ready */
    while (retry_count--) {
        ret = paw32xx_read_reg(dev, PAW32XX_PRODUCT_ID1, &val);
        if (ret < 0) {
            if (retry_count == 0) {
                return ret;
            }
            k_sleep(K_MSEC(100)); /* Wait before retrying */
            continue;
        }

        if (val != PRODUCT_ID_PAW32XX) {
            LOG_ERR("Invalid product id: %02x", val);

            if (retry_count == 0) {
                return -ENODEV; /* Device not ready after retries */
            }
            k_sleep(K_MSEC(100)); /* Wait before retrying */
            continue;
        } else {
            break; /* Device is ready */
        }
    }

    ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_RESET, CONFIGURATION_RESET);
    if (ret < 0) {
        return ret;
    }

    k_sleep(K_MSEC(RESET_DELAY_MS));

    if (cfg->res_cpi > 0) {
        paw32xx_set_resolution(dev, cfg->res_cpi);
    }

    paw32xx_force_awake(dev, cfg->force_awake);

    /* Dummy reads to clear any residual data */
    paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_X, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_Y, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_XY_HI, &val);

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int paw32xx_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct paw32xx_config *cfg = dev->config;
    int ret;
    uint8_t val;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        /* Disable IRQ interrupt */
        ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
        if (ret < 0) {
            LOG_ERR("Failed to disable IRQ interrupt: %d", ret);
            return ret;
        }

        /* Disconnect IRQ GPIO */
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_DISCONNECTED);
        if (ret < 0) {
            LOG_ERR("Failed to disconnect IRQ GPIO: %d", ret);
            return ret;
        }

        val = CONFIGURATION_PD_ENH;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_DISCONNECTED);
            if (ret < 0) {
                LOG_ERR("Failed to disconnect power: %d", ret);
                return ret;
            }
        }
#endif
        break;

    case PM_DEVICE_ACTION_RESUME:
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        if (gpio_is_ready_dt(&cfg->power_gpio)) {
            ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
            if (ret < 0) {
                LOG_ERR("Failed to enable power: %d", ret);
                return ret;
            }
            /* Wait for power stabilization */
            k_sleep(K_MSEC(10));
        }
#endif

        val = 0;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }

        /* Reconfigure IRQ GPIO as input */
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
            return ret;
        }

        /* Re-enable IRQ interrupt */
        ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to enable IRQ interrupt: %d", ret);
            return ret;
        }

#ifdef CONFIG_PAW3222_SCROLL_ACCEL
        /* reset accel state after resume to avoid spikes */
        {
            struct paw32xx_data *pdata = dev->data;
            pdata->last_scroll_time = 0;
            pdata->scroll_delta_x = 0;
            pdata->scroll_delta_y = 0;
            pdata->last_remainder_time = 0;
        }
#endif
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

/* Minimal device init to satisfy DEVICE_DT_INST_DEFINE and hook up handlers.
 * Placed after paw32xx_configure.
 */
static int paw32xx_init(const struct device *dev)
{
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    int ret;

    LOG_DBG("paw32xx_init called for %p", dev);

    /* store device pointer */
    data->dev = dev;

    /* init work and timer */
    k_work_init(&data->motion_work, paw32xx_motion_work_handler);
    k_timer_init(&data->motion_timer, paw32xx_motion_timer_handler, NULL);

    /* prepare gpio callback structure (callback not yet registered) */
    gpio_init_callback(&data->motion_cb, paw32xx_motion_handler,
                       BIT(cfg->irq_gpio.pin));

#ifdef CONFIG_PAW3222_SCROLL_ACCEL
    /* initialize accel state */
    data->last_scroll_time = 0;
    data->scroll_delta_x = 0;
    data->scroll_delta_y = 0;
    data->last_remainder_time = 0;
#endif

    /* If board provides a power GPIO, enable it now */
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("power_gpio configure failed: %d", ret);
            return ret;
        }
        LOG_DBG("power_gpio set ACTIVE, waiting 10ms");
        k_sleep(K_MSEC(10));
    } else {
        LOG_DBG("power_gpio not ready or not provided");
    }
#else
    LOG_DBG("no power_gpios property on DT instance");
#endif

    /* Configure IRQ GPIO if available */
    if (gpio_is_ready_dt(&cfg->irq_gpio)) {
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure IRQ gpio: %d", ret);
            return ret;
        }
        LOG_DBG("irq gpio configured as input");

        ret = gpio_add_callback(cfg->irq_gpio.port, &data->motion_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add irq callback: %d", ret);
            return ret;
        }
        LOG_DBG("irq callback added");

        ret = gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure IRQ interrupt: %d", ret);
            return ret;
        }
        LOG_DBG("irq interrupt configured (EDGE_TO_ACTIVE)");
    } else {
        LOG_DBG("irq_gpio not ready or not provided");
    }

    /* Quick SPI probe: read PRODUCT ID to confirm comms */
    {
        uint8_t id = 0;
        ret = paw32xx_read_reg(dev, PAW32XX_PRODUCT_ID1, &id);
        LOG_DBG("paw32xx_read_reg(PRODUCT_ID1) ret=%d id=0x%02x", ret, id);
        if (ret < 0) {
            LOG_ERR("SPI probe failed (read product id): %d", ret);
            return ret;
        }
        if (id != PRODUCT_ID_PAW32XX) {
            LOG_WRN("Unexpected product id: 0x%02x (expected 0x%02x)", id, PRODUCT_ID_PAW32XX);
            /* still continue so we can see behavior, or return -ENODEV to fail init */
        } else {
            LOG_INF("PAW32xx detected (0x%02x)", id);
        }
    }

    /* Run device configure sequence (SPI reads/writes etc) */
    ret = paw32xx_configure(dev);
    if (ret < 0) {
        LOG_ERR("paw32xx configure failed: %d", ret);
        return ret;
    }

    LOG_DBG("paw32xx initialized");
    return 0;
}

#define PAW32XX_SPI_MODE                                                                           \
    (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PAW32XX_INIT(n)                                                                            \
    BUILD_ASSERT(IN_RANGE(DT_INST_PROP_OR(n, res_cpi, RES_MIN), RES_MIN, RES_MAX),                 \
                 "invalid res-cpi");                                                               \
                                                                                                   \
    static const struct paw32xx_config paw32xx_cfg_##n = {                                         \
        .spi = SPI_DT_SPEC_INST_GET(n, PAW32XX_SPI_MODE, 0),                                       \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .power_gpio = GPIO_DT_SPEC_INST_GET_OR(n, power_gpios, {0}),                               \
        .res_cpi = DT_INST_PROP_OR(n, res_cpi, -1),                                                \
        .force_awake = DT_INST_PROP(n, force_awake),                                               \
    };                                                                                             \
                                                                                                   \
    static struct paw32xx_data paw32xx_data_##n;                                                   \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, paw32xx_pm_action);                                                \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, paw32xx_init, PM_DEVICE_DT_INST_GET(n), &paw32xx_data_##n,            \
                          &paw32xx_cfg_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PAW32XX_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
