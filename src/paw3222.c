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
#include <math.h>

#ifdef CONFIG_PAW3222_SMART_SCROLL
#include <zmk/keymap.h>
#endif

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#if defined(CONFIG_SOC_SERIES_NRF52X)
#include <hal/nrf_gpio.h>
#include <hal/nrf_spim.h>
#endif

#include "../include/paw3222.h"

LOG_MODULE_REGISTER(paw32xx, CONFIG_ZMK_LOG_LEVEL);

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

struct paw32xx_data {
    const struct device *dev;
    struct k_work motion_work;
    struct gpio_callback motion_cb;
    struct k_timer motion_timer; // Add timer for delayed motion checking
    atomic_t suspended;
#if defined(CONFIG_SOC_SERIES_NRF52X)
    NRF_SPIM_Type *spim;
    uint32_t spim_mosi_psel;
    uint32_t spim_miso_psel;
    uint32_t spim_sclk_psel;
    bool spim_mosi_psel_saved;
    bool spim_miso_psel_saved;
#endif
#ifdef CONFIG_PAW3222_SMART_SCROLL
    float remainder_x;        /* sub-integer accumulator for X output */
    float remainder_y;        /* sub-integer accumulator for Y output */
    int64_t last_motion_time; /* timestamp of the previous motion event (ms) */
    int64_t last_active_time; /* timestamp when remainders were last updated */
#endif
};

#define PAW32XX_NRF_PSEL_CONNECT_BIT BIT(31)
#define PAW32XX_NRF_PSEL_PIN_MASK GENMASK(4, 0)
#define PAW32XX_NRF_PSEL_PORT_BIT BIT(5)

static NRF_SPIM_Type *paw32xx_nrf52_spim_from_bus(const struct device *dev) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    const struct paw32xx_config *cfg = dev->config;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi0), okay) && defined(NRF_SPIM0)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi0))) {
        return NRF_SPIM0;
    }
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi1), okay) && defined(NRF_SPIM1)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi1))) {
        return NRF_SPIM1;
    }
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi2), okay) && defined(NRF_SPIM2)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi2))) {
        return NRF_SPIM2;
    }
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi3), okay) && defined(NRF_SPIM3)
    if (cfg->spi.bus == DEVICE_DT_GET(DT_NODELABEL(spi3))) {
        return NRF_SPIM3;
    }
#endif
    return NULL;
#else
    ARG_UNUSED(dev);
    return NULL;
#endif
}

static void paw32xx_nrf52_spim_deactivate(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim != NULL) {
        nrf_spim_disable(data->spim);
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_nrf52_spim_activate(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim != NULL) {
        nrf_spim_enable(data->spim);
    }
#else
    ARG_UNUSED(data);
#endif
}

static uint32_t paw32xx_nrf52_psel_to_pin(uint32_t psel) {
    uint32_t pin = psel & PAW32XX_NRF_PSEL_PIN_MASK;

    if ((psel & PAW32XX_NRF_PSEL_PORT_BIT) != 0U) {
        pin += 32U;
    }

    return pin;
}

static void paw32xx_sdio_init(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    data->spim_mosi_psel = data->spim->PSEL.MOSI;
    data->spim_miso_psel = data->spim->PSEL.MISO;
    data->spim_sclk_psel = data->spim->PSEL.SCK;

    data->spim_mosi_psel_saved = ((data->spim_mosi_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    data->spim_miso_psel_saved = ((data->spim_miso_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
    }
    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel));
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_PULLUP);
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_sdio_disconnect(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    if (!data->spim_mosi_psel_saved) {
        data->spim_mosi_psel = data->spim->PSEL.MOSI;
        data->spim_mosi_psel_saved = ((data->spim_mosi_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    }
    if (!data->spim_miso_psel_saved) {
        data->spim_miso_psel = data->spim->PSEL.MISO;
        data->spim_miso_psel_saved = ((data->spim_miso_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U);
    }

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_default(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
        data->spim->PSEL.MOSI = data->spim_mosi_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    }

    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_PULLUP);
        data->spim->PSEL.MISO = data->spim_miso_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_sdio_connect(struct paw32xx_data *data) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    if (data->spim == NULL) {
        return;
    }

    if (data->spim_mosi_psel_saved) {
        nrf_gpio_cfg_output(paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel));
        data->spim->PSEL.MOSI = data->spim_mosi_psel & ~PAW32XX_NRF_PSEL_CONNECT_BIT;
    }

    if (data->spim_miso_psel_saved) {
        nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel), NRF_GPIO_PIN_NOPULL);
        data->spim->PSEL.MISO = data->spim_miso_psel & ~PAW32XX_NRF_PSEL_CONNECT_BIT;
    }
#else
    ARG_UNUSED(data);
#endif
}

static void paw32xx_spi_transaction_begin(const struct device *dev) {
    struct paw32xx_data *data = dev->data;

    paw32xx_sdio_connect(data);
    paw32xx_nrf52_spim_activate(data);
}

static void paw32xx_spi_transaction_end(const struct device *dev) {
    struct paw32xx_data *data = dev->data;

    paw32xx_sdio_disconnect(data);
    paw32xx_nrf52_spim_deactivate(data);
}

static int paw32xx_force_cs(const struct device *dev, bool force_low) {
    const struct paw32xx_config *cfg = dev->config;
    const struct gpio_dt_spec *cs = NULL;
    int ret;

    if (cfg->spi.config.cs.gpio.port != NULL) {
        cs = &cfg->spi.config.cs.gpio;
    }

    if (cs == NULL || cs->port == NULL || !device_is_ready(cs->port)) {
        LOG_ERR("CS GPIO not defined or not ready");
        return -ENODEV;
    }

    ret = gpio_pin_set_dt(cs, force_low ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to drive CS pin: %d", ret);
        return ret;
    }

    return 0;
}

// Define a custom sign_extend function to avoid conflict with Zephyr's implementation
static inline int32_t _sign_extend(uint32_t value, uint8_t index) {
    __ASSERT_NO_MSG(index <= 31);

    uint8_t shift = 31 - index;

    return (int32_t)(value << shift) >> shift;
}

#if defined(CONFIG_SOC_SERIES_NRF52X)
/* Half-duplex SDIO: release the output before the sensor sends data.
 * Uses pinctrl's configured pins, with the successful GPIO diagnostic timing. */
static int paw32xx_gpio_transfer(const struct device *dev, uint8_t addr,
                                uint8_t *value, bool write) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    if (!data->spim_mosi_psel_saved || !data->spim_miso_psel_saved) {
        return -EINVAL;
    }
    uint32_t sd = paw32xx_nrf52_psel_to_pin(data->spim_mosi_psel);
    uint32_t clk = paw32xx_nrf52_psel_to_pin(data->spim_sclk_psel);
    if (sd != paw32xx_nrf52_psel_to_pin(data->spim_miso_psel)) {
        return -EINVAL;
    }
    paw32xx_nrf52_spim_deactivate(data);
    paw32xx_sdio_disconnect(data);
    data->spim->PSEL.SCK = data->spim_sclk_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    nrf_gpio_pin_set(clk);
    nrf_gpio_cfg_output(clk);
    int ret = gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) { return ret; }
    ret = paw32xx_force_cs(dev, true);
    if (ret < 0) { return ret; }
    k_busy_wait(20);
    nrf_gpio_cfg_output(sd);
    uint8_t command = write ? (addr | SPI_WRITE) : (addr & 0x7f);
    for (int bit = 7; bit >= 0; bit--) {
        nrf_gpio_pin_write(sd, (command >> bit) & 1);
        k_busy_wait(50);
        nrf_gpio_pin_clear(clk);
        k_busy_wait(50);
        nrf_gpio_pin_set(clk);
        k_busy_wait(50);
    }
    if (!write) {
        nrf_gpio_cfg_input(sd, NRF_GPIO_PIN_NOPULL);
        *value = 0;
        k_busy_wait(300);
    }
    for (int bit = 7; bit >= 0; bit--) {
        if (write) { nrf_gpio_pin_write(sd, (*value >> bit) & 1); }
        nrf_gpio_pin_clear(clk);
        k_busy_wait(50);
        if (!write) {
            /* The sensor drives each bit after the falling clock edge.
             * Sample while CLK is low, before the next rising edge. */
            *value |= nrf_gpio_pin_read(sd) << bit;
        }
        nrf_gpio_pin_set(clk);
        k_busy_wait(50);
    }
    ret = paw32xx_force_cs(dev, false);
    nrf_gpio_cfg_input(sd, NRF_GPIO_PIN_NOPULL);
    k_busy_wait(20);
    return ret;
}
#endif

static int paw32xx_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    return paw32xx_gpio_transfer(dev, addr, value, false);
#else
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

    paw32xx_spi_transaction_begin(dev);
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    paw32xx_spi_transaction_end(dev);

    return ret;
#endif
}

static int paw32xx_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
#if defined(CONFIG_SOC_SERIES_NRF52X)
    return paw32xx_gpio_transfer(dev, addr, &value, true);
#else
    const struct paw32xx_config *cfg = dev->config;
    int ret;

    uint8_t write_buf[] = {addr | SPI_WRITE, value};
    const struct spi_buf tx_buf = {
        .buf = write_buf,
        .len = sizeof(write_buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };

    paw32xx_spi_transaction_begin(dev);
    ret = spi_write_dt(&cfg->spi, &tx);
    paw32xx_spi_transaction_end(dev);

    return ret;
#endif
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
#if defined(CONFIG_SOC_SERIES_NRF52X)
    uint8_t dx, dy;
    int ret = paw32xx_read_reg(dev, PAW32XX_DELTA_X, &dx);
    if (ret < 0) { return ret; }
    ret = paw32xx_read_reg(dev, PAW32XX_DELTA_Y, &dy);
    if (ret < 0) { return ret; }
    *x = dx;
    *y = dy;
#else
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

    paw32xx_spi_transaction_begin(dev);
    ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
    paw32xx_spi_transaction_end(dev);
    if (ret < 0) {
        return ret;
    }

    *x = rx_data[1];
    *y = rx_data[3];
#endif

    *x = _sign_extend(*x, PAW32XX_DATA_SIZE_BITS - 1);
    *y = _sign_extend(*y, PAW32XX_DATA_SIZE_BITS - 1);

    return 0;
}

static int paw32xx_interrupt_configure(const struct device *dev, gpio_flags_t flags) {
    const struct paw32xx_config *cfg = dev->config;

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        return -ENODEV;
    }

    return gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, flags);
}

static int paw32xx_interrupt_enable(const struct device *dev) {
    return paw32xx_interrupt_configure(dev, GPIO_INT_LEVEL_LOW);
}

static int paw32xx_interrupt_disable(const struct device *dev) {
    return paw32xx_interrupt_configure(dev, GPIO_INT_DISABLE);
}

static void paw32xx_motion_timer_handler(struct k_timer *timer) {
    struct paw32xx_data *data = CONTAINER_OF(timer, struct paw32xx_data, motion_timer);

    if (atomic_get(&data->suspended)) {
        return;
    }

    k_work_submit(&data->motion_work);
}

static void paw32xx_reenable_motion_interrupt(const struct device *dev) {
    struct paw32xx_data *data = dev->data;
    int ret;

    if (atomic_get(&data->suspended)) {
        return;
    }

    ret = paw32xx_interrupt_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to re-enable motion interrupt: %d", ret);
    }
}

static void paw32xx_motion_work_handler(struct k_work *work) {
    struct paw32xx_data *data = CONTAINER_OF(work, struct paw32xx_data, motion_work);
    const struct device *dev = data->dev;
    const struct paw32xx_config *cfg = dev->config;
    uint8_t val;
    int16_t x, y;
    int ret;

    if (atomic_get(&data->suspended)) {
        return;
    }

    ret = paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    if (ret < 0) {
        LOG_WRN("Failed to read motion register: %d", ret);
        paw32xx_reenable_motion_interrupt(dev);
        return;
    }

    if ((val & MOTION_STATUS_MOTION) == 0x00) {
        // No motion detected, re-enable interrupts and wait for next interrupt
        paw32xx_reenable_motion_interrupt(dev);

        ret = gpio_pin_get_dt(&cfg->irq_gpio);
        if (ret <= 0) {
            if (ret < 0) {
                LOG_WRN("Failed to read motion GPIO: %d", ret);
            }
            return;
        }
    }

    ret = paw32xx_read_xy(dev, &x, &y);
    if (ret < 0) {
        LOG_WRN("Failed to read motion delta: %d", ret);
        paw32xx_reenable_motion_interrupt(dev);
        return;
    }

    LOG_DBG("x=%4d y=%4d", x, y);

#ifdef CONFIG_PAW3222_SMART_SCROLL
    uint8_t current_layer = zmk_keymap_highest_layer_active();
    if (current_layer == CONFIG_PAW3222_SMART_SCROLL_LAYER) {
        int64_t now = k_uptime_get();

        /* ---- 残余バッファの時間リセット ----
         * 100 ms 以上入力が途絶えた場合、小数点以下の残余値を
         * ゼロクリアする。方向転換時のバッファリング問題を防ぐ。 */
        if (data->last_active_time > 0 &&
            (now - data->last_active_time) > 100) {
            data->remainder_x = 0.0f;
            data->remainder_y = 0.0f;
        }

        /* ---- シグモイド加速 ----
         * speed = 移動量 / 経過時間(ms)。
         * 加速係数: 1.0 (遅い) 〜 10.0 (速い) のシグモイド曲線。 */
        float accel = 1.0f;
        int64_t delta_ms = (data->last_motion_time > 0)
                           ? now - data->last_motion_time : 0;
        if (delta_ms > 0 && delta_ms < 100) {
            float speed = (float)(abs(x) + abs(y)) / (float)delta_ms;
            float sens  = (float)CONFIG_PAW3222_SCROLL_SENSITIVITY / 2.5f;
            /* sigmoid: 1 + 4 / (1 + exp(-0.3*(speed-8))) 
             * 最大加速を約5倍(元は10倍)に抑え、立ち上がりの傾き(-0.5 -> -0.3)も緩やかに */
            accel = (1.0f + 4.0f * (1.0f / (1.0f + expf(-0.3f * (speed - 8.0f)))))
                    * sens;
        }
        data->last_motion_time = now;

        /* ---- ベース感度スケーリング + 残余蓄積 ---- */
        float base = (float)CONFIG_PAW3222_BASE_SENSITIVITY_PERCENT / 100.0f;
        data->remainder_x += (float)x * accel * base;
        data->remainder_y += (float)y * accel * base;
        data->last_active_time = now;

        /* 整数部分を切り出してイベント送信 */
        int16_t out_x = (int16_t)data->remainder_x;
        int16_t out_y = (int16_t)data->remainder_y;
        data->remainder_x -= (float)out_x;
        data->remainder_y -= (float)out_y;

        if (out_x != 0 || out_y != 0) {
            /* 最後のイベントに sync=true を立てる */
            bool x_is_last = (out_y == 0);
            if (out_x != 0) {
                input_report_rel(data->dev, INPUT_REL_X, out_x, x_is_last, K_FOREVER);
            }
            if (out_y != 0) {
                input_report_rel(data->dev, INPUT_REL_Y, out_y, true, K_FOREVER);
            }
        }
    } else {
        /* 通常レイヤー（ポインタ移動）：スケーリングなし、そのまま出力 */
        input_report_rel(data->dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);
    }
#else
    input_report_rel(data->dev, INPUT_REL_X, x, false, K_FOREVER);
    input_report_rel(data->dev, INPUT_REL_Y, y, true, K_FOREVER);
#endif

    // Schedule the next check after the configured delay.
    if (!atomic_get(&data->suspended)) {
        k_timer_start(&data->motion_timer, K_MSEC(CONFIG_PAW3222_MOTION_INTERVAL_MS), K_NO_WAIT);
    }
}

static void paw32xx_motion_handler(const struct device *gpio_dev, struct gpio_callback *cb,
                                   uint32_t pins) {
    struct paw32xx_data *data = CONTAINER_OF(cb, struct paw32xx_data, motion_cb);
    const struct device *dev = data->dev;

    ARG_UNUSED(gpio_dev);
    ARG_UNUSED(pins);

    if (atomic_get(&data->suspended)) {
        return;
    }

    // Disable interrupts while timer is active
    paw32xx_interrupt_disable(dev);

    // Cancel any pending timer
    k_timer_stop(&data->motion_timer);

    // Process motion
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

    // Check if the device is ready
    while (retry_count--) {
        ret = paw32xx_read_reg(dev, PAW32XX_PRODUCT_ID1, &val);
        if (ret < 0) {
            if (retry_count == 0) {
                return ret;
            }
            k_sleep(K_MSEC(100)); // Wait before retrying
            continue;
        }

        if (val != PRODUCT_ID_PAW32XX) {
            LOG_ERR("Invalid product id: %02x", val);

            if (retry_count == 0) {
                return -ENODEV; // Device not ready after retries
            }
#if DT_INST_NODE_HAS_PROP(0, power_gpios)
            // reboot
            ret = paw32xx_force_cs(dev, true);
            if (ret < 0) {
                return ret;
            }

            gpio_pin_set_dt(&cfg->power_gpio, 0);
            k_sleep(K_MSEC(50)); // Wait before retrying
            gpio_pin_set_dt(&cfg->power_gpio, 1);

            ret = paw32xx_force_cs(dev, false);
            if (ret < 0) {
                return ret;
            }
#endif
            k_sleep(K_MSEC(100)); // Wait before retrying
            continue;
        }
        else {
            break; // Device is ready
        }
    }

    ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_RESET, CONFIGURATION_RESET);
    if (ret < 0) {
        return ret;
    }

    k_sleep(K_MSEC(RESET_DELAY_MS));

    if (cfg->res_cpi > 0) {
        ret = paw32xx_set_resolution(dev, cfg->res_cpi);
        if (ret < 0) {
            return ret;
        }
    }

    ret = paw32xx_force_awake(dev, cfg->force_awake);
    if (ret < 0) {
        return ret;
    }

    // Dummy reads to clear any residual data
    paw32xx_read_reg(dev, PAW32XX_MOTION, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_X, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_Y, &val);
    paw32xx_read_reg(dev, PAW32XX_DELTA_XY_HI, &val);

    return 0;
}

static int paw32xx_init(const struct device *dev) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    int ret;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("%s is not ready", cfg->spi.bus->name);
        return -ENODEV;
    }

#if defined(CONFIG_SOC_SERIES_NRF52X)
    data->spim = paw32xx_nrf52_spim_from_bus(dev);
    if (data->spim == NULL) {
        LOG_ERR("Unsupported nRF52 SPIM controller: %s", cfg->spi.bus->name);
        return -ENOTSUP;
    }
#endif

    paw32xx_sdio_init(data);
    paw32xx_sdio_disconnect(data);

    data->dev = dev;
    atomic_clear(&data->suspended);

    k_work_init(&data->motion_work, paw32xx_motion_work_handler);
    // Initialize the timer for delayed motion checks
    k_timer_init(&data->motion_timer, paw32xx_motion_timer_handler, NULL);

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
    // Initialize power GPIO if defined
    if (gpio_is_ready_dt(&cfg->power_gpio)) {
        ret = paw32xx_force_cs(dev, true);
        if (ret != 0) {
            return ret;
        }

        // Configure as output but start with power OFF
        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Power pin configuration failed: %d", ret);
            return ret;
        }

        // Wait 0.01 seconds before turning on power
        k_sleep(K_MSEC(10));

        // Now turn on power
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret != 0) {
            LOG_ERR("Power pin set failed: %d", ret);
            return ret;
        }

        // Wait for power stabilization
        k_sleep(K_MSEC(500));

        ret = paw32xx_force_cs(dev, false);
        if (ret != 0) {
            return ret;
        }

        // Wait for power stabilization
        k_sleep(K_MSEC(50));
    }
#endif

    if (!gpio_is_ready_dt(&cfg->irq_gpio)) {
        LOG_ERR("%s is not ready", cfg->irq_gpio.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Motion pin configuration failed: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->motion_cb, paw32xx_motion_handler, BIT(cfg->irq_gpio.pin));

    ret = gpio_add_callback_dt(&cfg->irq_gpio, &data->motion_cb);
    if (ret < 0) {
        LOG_ERR("Could not set motion callback: %d", ret);
        return ret;
    }

#if defined(CONFIG_SOC_SERIES_NRF52X)
    /* Reset clock/bus state before the first product-ID transaction. */
    paw32xx_nrf52_spim_deactivate(data);
    data->spim->PSEL.SCK = data->spim_sclk_psel | PAW32XX_NRF_PSEL_CONNECT_BIT;
    uint32_t clk = paw32xx_nrf52_psel_to_pin(data->spim_sclk_psel);
    ret = gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) { return ret; }
    nrf_gpio_pin_clear(clk);
    nrf_gpio_cfg_output(clk);
    k_busy_wait(1000);
    nrf_gpio_pin_set(clk);
    k_busy_wait(2000);
#endif

    ret = paw32xx_configure(dev);
    if (ret != 0) {
        LOG_ERR("Device configuration failed: %d", ret);
        return ret;
    }

    ret = paw32xx_interrupt_enable(dev);
    if (ret != 0) {
        LOG_ERR("Motion interrupt configuration failed: %d", ret);
        return ret;
    }

    ret = pm_device_runtime_enable(dev);
    if (ret < 0) {
        LOG_ERR("Failed to enable runtime power management: %d", ret);
        return ret;
    }

    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int paw32xx_pm_action(const struct device *dev, enum pm_device_action action) {
    const struct paw32xx_config *cfg = dev->config;
    struct paw32xx_data *data = dev->data;
    struct k_work_sync sync;
    int ret;
    uint8_t val;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        atomic_set(&data->suspended, 1);

        // Disable IRQ interrupt
        ret = paw32xx_interrupt_disable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to disable IRQ interrupt: %d", ret);
            atomic_clear(&data->suspended);
            return ret;
        }

        // Drain motion processing before changing SPI and sensor power state.
        k_timer_stop(&data->motion_timer);
        k_work_cancel_sync(&data->motion_work, &sync);
        k_timer_stop(&data->motion_timer);

        // Disconnect IRQ GPIO
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_DISCONNECTED);
        if (ret < 0) {
            LOG_ERR("Failed to disconnect IRQ GPIO: %d", ret);
            atomic_clear(&data->suspended);
            paw32xx_reenable_motion_interrupt(dev);
            return ret;
        }

        val = CONFIGURATION_PD_ENH;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
            atomic_clear(&data->suspended);
            paw32xx_reenable_motion_interrupt(dev);
            return ret;
        }

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        // Power off the device
        gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_INPUT | GPIO_PULL_DOWN);
#if defined(CONFIG_SOC_SERIES_NRF52X)
        if (data->spim != NULL) {
            if (data->spim_miso_psel_saved) {
                nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_miso_psel),
                                   NRF_GPIO_PIN_PULLDOWN);
            }
            if ((data->spim_sclk_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U) {
                nrf_gpio_cfg_input(paw32xx_nrf52_psel_to_pin(data->spim_sclk_psel),
                                   NRF_GPIO_PIN_PULLDOWN);
            }
        }
#endif
        gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
        gpio_pin_configure_dt(&cfg->power_gpio, GPIO_INPUT | GPIO_PULL_DOWN);
#endif

        break;

    case PM_DEVICE_ACTION_RESUME:

#if DT_INST_NODE_HAS_PROP(0, power_gpios)
        ret = gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to restore CS GPIO: %d", ret);
            return ret;
        }

#if defined(CONFIG_SOC_SERIES_NRF52X)
        if (data->spim != NULL &&
            (data->spim_sclk_psel & PAW32XX_NRF_PSEL_CONNECT_BIT) == 0U) {
            nrf_gpio_cfg_output(paw32xx_nrf52_psel_to_pin(data->spim_sclk_psel));
        }
#endif

        ret = gpio_pin_configure_dt(&cfg->power_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to restore power GPIO: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(10));
        ret = gpio_pin_set_dt(&cfg->power_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to enable sensor power: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(500));

        ret = paw32xx_configure(dev);
        if (ret < 0) {
            LOG_ERR("Failed to configure device after resume: %d", ret);
            return ret;
        }
#endif

        val = 0;
        ret = paw32xx_update_reg(dev, PAW32XX_CONFIGURATION, CONFIGURATION_PD_ENH, val);
        if (ret < 0) {
            return ret;
        }

        // Reconfigure IRQ GPIO as input
        ret = gpio_pin_configure_dt(&cfg->irq_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure IRQ GPIO: %d", ret);
            return ret;
        }

        // Clear the guard before enabling the level interrupt so an immediate
        // callback can submit motion work.
        atomic_clear(&data->suspended);

        // Re-enable IRQ interrupt
        ret = paw32xx_interrupt_enable(dev);
        if (ret < 0) {
            LOG_ERR("Failed to enable IRQ interrupt: %d", ret);
            atomic_set(&data->suspended, 1);
            return ret;
        }
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif

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

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
