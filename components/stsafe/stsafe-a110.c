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
 * @file stsafe-a110.c
 *
 * ESP-IDF driver for stsafe
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "stsafe-a110.h"

#define I2C_FREQ_HZ 400000

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static inline esp_err_t read_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_dev_read_reg(dev, reg, val, 1);
}

static inline esp_err_t write_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    return i2c_dev_write_reg(dev, reg, &val, 1);
}

static esp_err_t update_reg_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    CHECK(read_reg_nolock(dev, reg, &v));
    CHECK(write_reg_nolock(dev, reg, (v & ~mask) | val));
    return ESP_OK;
}

static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, write_reg_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static esp_err_t update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, update_reg_nolock(dev, reg, mask, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t stsafe_a110_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = STSAFE_A110_I2C_ADDR;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(dev);
}

esp_err_t stsafe_a110_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}
