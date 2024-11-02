/*
 * Copyright (c) 2024 WH Wong
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
 * @file lib_st25dvxxkc.c
 *
 * ESP-IDF driver for st25dvxxkc
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

#include "lib_st25dvxxkc.h"

#define DEBUG_EN 0

#if (DEBUG_EN > 0)
static const char *TAG = "ST25DVxxKC";
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

static i2c_dev_t *p_dev = NULL;

///////////////////////////////////////////////////////////////////////////////
esp_err_t st25dvxxkc_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = ST25DVxxKC_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    CHECK(i2c_dev_create_mutex(dev));

    p_dev = dev;
#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "st25dvxxkc_init_desc successful");
#endif
    return ESP_OK;
}

esp_err_t st25dvxxkc_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

/**
 * @brief
 *
 * @return int32_t
 */
int32_t BSP_I2C1_Init(void)
{
    /* this function always return OK in this environment as i2c dev is initialised before any st25dv app is called */
    return NFCTAG_OK;
}

/**
 * @brief
 *
 * @return int32_t
 */
int32_t BSP_I2C1_DeInit(void)
{
    esp_err_t ret = ESP_OK;
    if (p_dev == NULL)
        ret = ESP_FAIL;
    else
        ret = st25dvxxkc_free_desc(p_dev);

    if (ret != ESP_OK)
        ret = ESP_FAIL;
    return ret;
}

/**
 * @brief
 *
 * @param DevAddr
 * @param Trials
 * @return int32_t
 */
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
    (void)Trials;
    esp_err_t ret = ESP_OK;
    if (p_dev == NULL)
        ret = ESP_FAIL;
    else
        ret = i2c_dev_probe_with_addr(p_dev, (uint8_t)DevAddr, I2C_DEV_READ);
#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "BSP_I2C1_IsReady[%x]: %s", DevAddr, esp_err_to_name(ret));
#endif

    if (ret != ESP_OK)
        ret = ESP_FAIL;
    return ret;
}

/**
 * @brief
 *
 * @param DevAddr
 * @param Reg
 * @param pData
 * @param Length
 * @return int32_t
 */
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
    esp_err_t ret = ESP_OK;
    Reg = BYTE_SWAP_16(Reg);
    if (p_dev == NULL)
        return ESP_FAIL;

    ret = i2c_dev_write_with_address(p_dev, (uint8_t)DevAddr, (uint8_t *)&Reg, sizeof(Reg), pData, Length);

#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "BSP_I2C1_WriteReg16[%x][%x]: %s", DevAddr, Reg, esp_err_to_name(ret));
    ESP_LOG_BUFFER_HEX(TAG, pData, Length);
#endif

    if (ret != ESP_OK)
        ret = ESP_FAIL;

    return ret;
}

/**
 * @brief
 *
 * @param DevAddr
 * @param Reg
 * @param pData
 * @param Length
 * @return int32_t
 */
int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
    esp_err_t ret = ESP_OK;
    Reg = BYTE_SWAP_16(Reg);
    if (p_dev == NULL)
        return ESP_FAIL;

    ret = i2c_dev_read_with_address(p_dev, (uint8_t)DevAddr, (uint8_t *)&Reg, sizeof(Reg), pData, Length);
#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "BSP_I2C1_ReadReg16[%x][%x]: %s", DevAddr, Reg, esp_err_to_name(ret));
    ESP_LOG_BUFFER_HEX(TAG, pData, Length);
#endif
    if (ret != ESP_OK)
        ret = ESP_FAIL;

    return ret;
}

/**
 * @brief
 *
 * @param DevAddr
 * @param pData
 * @param Length
 * @return int32_t
 */
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
    esp_err_t ret = ESP_OK;
    if (p_dev == NULL)
        return ESP_FAIL;

    ret = i2c_dev_read_with_address(p_dev, DevAddr, NULL, 0, pData, Length);
#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "BSP_I2C1_Recv[%x]: %s", DevAddr, esp_err_to_name(ret));
    ESP_LOG_BUFFER_HEX(TAG, pData, Length);
#endif
    if (ret != ESP_OK)
        ret = ESP_FAIL;

    return ret;
}
