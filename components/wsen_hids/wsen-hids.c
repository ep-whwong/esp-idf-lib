/*
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file wsen-hids.c
 *
 * ESP-IDF driver for WSEN HIDS temperature and humidity sensor
 *
 * Copyright (c) 2024 WH Wong
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>

#include <esp_log.h>
#include "wsen-hids.h"
#include "WeSensorsSDK.h"
#include "WSEN_HIDS_2525020210002.h"

#define DEBUG_EN 1

static WE_sensorInterface_t hids;

#if (DEBUG_EN > 0)
static const char *TAG = "WSEN_HIDS";
#endif

#define I2C_FREQ_HZ 400000
#define CHECK(x)                                                                                                                                                                                       \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        esp_err_t __;                                                                                                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                                                                                                        \
            return __;                                                                                                                                                                                 \
    }                                                                                                                                                                                                  \
    while (0)
#define CHECK_ARG(ARG)                                                                                                                                                                                 \
    do                                                                                                                                                                                                 \
    {                                                                                                                                                                                                  \
        if (!(ARG))                                                                                                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                                                                                                \
    }                                                                                                                                                                                                  \
    while (0)

///////////////////////////////////////////////////////////////////////////////

esp_err_t wsen_hids_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = WSEN_HIDS_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    CHECK(i2c_dev_create_mutex(dev));

    HIDS_Get_Default_Interface(&hids);
    hids.interfaceType = WE_i2c_fifo;
    hids.handle = dev;

    /* Wait for boot */
    vTaskDelay(pdMS_TO_TICKS(50));
    if (WE_SUCCESS != HIDS_Sensor_Init(&hids))
    {
#if (DEBUG_EN > 0)
        ESP_LOGI(TAG, "**** HIDS_MULTIPLEXER_Init error. STOP ****");
#endif
        vTaskDelay(pdMS_TO_TICKS(5));
        return ESP_FAIL;
    }

#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "**** WE_isSensorInterfaceReady(): OK ****");
#endif

    return ESP_OK;
}

esp_err_t wsen_hids_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t wsen_measure_raw(int32_t *p_temp_raw, int32_t *p_humidity_raw)
{
    CHECK_ARG(p_temp_raw);
    CHECK_ARG(p_humidity_raw);

    hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
    if (WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids, measureCmd, p_temp_raw, p_humidity_raw))
    {
#if (DEBUG_EN > 0)
        ESP_LOGI(TAG, "Hum: %li", *p_humidity_raw);
        ESP_LOGI(TAG, "Temp: %li", *p_temp_raw);
#endif

        return ESP_OK;
    }
    return ESP_FAIL;
}