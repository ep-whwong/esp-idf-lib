/*
 ***************************************************************************************************
 * This file is part of Sensors SDK:
 * https://www.we-online.com/sensors, https://github.com/WurthElektronik/Sensors-SDK_STM32
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES’ INTELLECTUAL PROPERTY RIGHTS OR THE
 * MERCHANTABILITY OR FITNESS FOR YOUR INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT
 * WARRANT OR REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY PATENT
 * RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY RIGHT RELATING TO ANY
 * COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT IS USED. INFORMATION PUBLISHED BY
 * WÜRTH ELEKTRONIK EISOS REGARDING THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE
 * FROM WÜRTH ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR ENDORSEMENT
 * THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE (license_terms_wsen_sdk.pdf)
 * LOCATED IN THE ROOT DIRECTORY OF THIS DRIVER PACKAGE.
 *
 * COPYRIGHT (c) 2022 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief Contains platform-specific functions.
 */

/* NOTE: https://forum.digikey.com/t/using-the-stm32cube-hal-i2c-driver-in-master-mode/15122  */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>

#include <esp_log.h>

#include "wsen-hids.h"
#include "platform.h"

#define DEBUG_EN 0

#if (DEBUG_EN > 0)
static const char *TAG = "WSEN_PLAT";
#endif

/**
 * @brief Reads bytes from I2C
 * @param[in] handle I2C handle
 * @param[in] addr I2C address
 * @param[in] reg Register address
 * @param[in] numBytesToRead Number of bytes to read
 * @param[in] slaveTransmitterMode Enables slave-transmitter mode (read-only, polling mode IO operation), slaveTransmitterMode = 1 is only required for WSEN-PDUS operation, other sensors use 0.
 * @param[in] timeout Timeout for read operation
 * @param[out] value Pointer to data buffer
 * @retval HAL status
 */
static esp_err_t I2Cx_ReadBytes(i2c_dev_t *handle, uint8_t addr, uint16_t reg, uint16_t numBytesToRead, uint8_t slaveTransmitterMode, uint16_t timeout, uint8_t *value)
{
    (void)timeout;
    esp_err_t ret = ESP_OK;
    uint8_t i2c_reg = (uint8_t)reg;

    ret = i2c_dev_take_mutex(handle);
    if (ret == ESP_OK)
    {
        if (slaveTransmitterMode == 0)
            ret = i2c_dev_read(handle, &i2c_reg, 1, value, numBytesToRead);
        else
            ret = i2c_dev_read(handle, NULL, 0, value, numBytesToRead);

        if (ret == ESP_OK)
            ret = i2c_dev_give_mutex(handle);
    }

#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "I2Cx_ReadBytes[%x][%x]: %s", addr, reg, esp_err_to_name(ret));
    if (ret == ESP_OK)
        ESP_LOG_BUFFER_HEX(TAG, value, numBytesToRead);
#endif

    return ret;
}

/**
 * @brief
 *
 * @param handle
 * @param addr
 * @param numBytesToWrite
 * @param timeout
 * @param value
 * @return esp_err_t
 */
static esp_err_t I2Cx_WriteBytesWOReg(i2c_dev_t *handle, uint8_t addr, uint16_t numBytesToWrite, uint16_t timeout, uint8_t *value)
{
    (void)timeout;
    esp_err_t ret = ESP_OK;

    ret = i2c_dev_take_mutex(handle);
    if (ret == ESP_OK)
    {
        ret = i2c_dev_write(handle, NULL, 0, value, numBytesToWrite);
        if (ret == ESP_OK)
            ret = i2c_dev_give_mutex(handle);
    }
#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "I2Cx_WriteBytesWOReg[%x]: %s", addr, esp_err_to_name(ret));
    if (ret == ESP_OK)
        ESP_LOG_BUFFER_HEX(TAG, value, numBytesToWrite);
#endif
    return ret;
}

/**
 * @brief Writes bytes to I2C.
 * @param[in] handle I2C handle
 * @param[in] addr I2C address
 * @param[in] reg The target register address to write
 * @param[in] numBytesToWrite Number of bytes to write
 * @param[in] timeout Timeout for write operation
 * @param[in] value The target register value to be written
 * @retval HAL status
 */
static esp_err_t I2Cx_WriteBytes(i2c_dev_t *handle, uint8_t addr, uint16_t reg, uint16_t numBytesToWrite, uint16_t timeout, uint8_t *value)
{
    (void)timeout;
    esp_err_t ret = ESP_OK;

    ret = i2c_dev_take_mutex(handle);
    if (ret == ESP_OK)
    {
        ret = i2c_dev_write(handle, &reg, 1, value, numBytesToWrite);
        if (ret == ESP_OK)
            ret = i2c_dev_give_mutex(handle);
    }

#if (DEBUG_EN > 0)
    ESP_LOGI(TAG, "I2Cx_WriteBytes[%x][%x]: %s", addr, reg, esp_err_to_name(ret));
    if (ret == ESP_OK)
        ESP_LOG_BUFFER_HEX(TAG, value, numBytesToWrite);
#endif

    return ret;
}

/**
 * @brief Read data starting from the addressed register
 * @param[in] interface Sensor interface
 * @param[in] regAdr The register address to read from
 * @param[in] numBytesToRead Number of bytes to read
 * @param[out] data The read data will be stored here
 * @retval Error code
 */
inline int8_t WE_ReadReg(WE_sensorInterface_t *interface, uint8_t regAdr, uint16_t numBytesToRead, uint8_t *data)
{
    esp_err_t status = ESP_OK;
    switch (interface->interfaceType)
    {
        case WE_i2c_fifo:
            if (interface->options.i2c.burstMode != 0 || numBytesToRead == 1)
            {
                if (numBytesToRead > 1 && interface->options.i2c.useRegAddrMsbForMultiBytesRead)
                {
                    /* Register address most significant bit is used to enable multi bytes read */
                    regAdr |= 1 << 7;
                }
                status = I2Cx_ReadBytes((i2c_dev_t *)interface->handle, interface->options.i2c.address, (uint16_t)regAdr, numBytesToRead, interface->options.i2c.slaveTransmitterMode,
                    interface->options.readTimeout, data);
            }
            break;

        case WE_i2c:
            if (interface->options.i2c.burstMode != 0 || numBytesToRead == 1)
            {
                if (numBytesToRead > 1 && interface->options.i2c.useRegAddrMsbForMultiBytesRead)
                {
                    /* Register address most significant bit is used to enable multi bytes read */
                    regAdr |= 1 << 7;
                }
                status = I2Cx_ReadBytes((i2c_dev_t *)interface->handle, interface->options.i2c.address, (uint16_t)regAdr, numBytesToRead, interface->options.i2c.slaveTransmitterMode,
                    interface->options.readTimeout, data);
            }
            else
            {
                for (uint16_t i = 0; (i < numBytesToRead) && (status == ESP_OK); i++)
                {
                    status = I2Cx_ReadBytes((i2c_dev_t *)interface->handle, interface->options.i2c.address, regAdr + i, 1, interface->options.i2c.slaveTransmitterMode, interface->options.readTimeout,
                        data + i);
                }
            }
            break;

        case WE_spi:
            status = ESP_FAIL;
            break;

        default:
            status = ESP_FAIL;
            break;
    }

    return (status == ESP_OK) ? WE_SUCCESS : WE_FAIL;
}

/**
 * @brief Write data starting from the addressed register
 * @param[in] interface Sensor interface
 * @param[in] regAdr Address of register to be written
 * @param[in] numBytesToWrite Number of bytes to write
 * @param[in] data Data to be written
 * @retval Error code
 */
inline int8_t WE_WriteReg(WE_sensorInterface_t *interface, uint8_t regAdr, uint16_t numBytesToWrite, uint8_t *data)
{
    esp_err_t status = ESP_OK;

    switch (interface->interfaceType)
    {
        case WE_i2c_fifo:
            status = I2Cx_WriteBytesWOReg(interface->handle, interface->options.i2c.address, numBytesToWrite, interface->options.writeTimeout, data);
            break;

        case WE_i2c:
            if (interface->options.i2c.burstMode != 0 || numBytesToWrite == 1)
            {
                status = I2Cx_WriteBytes((i2c_dev_t *)interface->handle, interface->options.i2c.address, regAdr, numBytesToWrite, interface->options.writeTimeout, data);
            }
            else
            {
                for (uint16_t i = 0; (i < numBytesToWrite) && (status == ESP_OK); i++)
                {
                    status = I2Cx_WriteBytes((i2c_dev_t *)interface->handle, interface->options.i2c.address, regAdr + i, 1, interface->options.writeTimeout, data + i);
                }
            }
            break;

        case WE_spi:
            status = ESP_FAIL;
            break;

        default:
            status = ESP_FAIL;
            break;
    }

    return (status == ESP_OK) ? WE_SUCCESS : WE_FAIL;
}

/**
 * @brief Checks if the sensor interface is ready.
 * @param[in] interface Sensor interface
 * @return WE_SUCCESS if interface is ready, WE_FAIL if not.
 */
int8_t WE_isSensorInterfaceReady(WE_sensorInterface_t *interface)
{
    switch (interface->interfaceType)
    {
        case WE_i2c:
            if (i2c_dev_probe((i2c_dev_t *)interface->handle, I2C_DEV_READ) != ESP_OK)
                return WE_FAIL;

            return WE_SUCCESS;
        case WE_spi:
            return WE_FAIL;
        default:
            return WE_FAIL;
    }
}

/**
 * @brief Provides delay
 * @param[in] Delay in milliseconds
 */
void WE_Delay(uint32_t Delay)
{
    vTaskDelay(pdMS_TO_TICKS(Delay));
}
