/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
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
 * @file wiegand.c
 *
 * ESP-IDF Wiegand protocol receiver
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <esp_idf_lib_helpers.h>
#include "wiegand.h"

static const char *TAG = "wiegand";

#define TIMER_INTERVAL_US 50000 // 50ms

#define CHECK(x)                                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        esp_err_t __;                                                                                                  \
        if ((__ = x) != ESP_OK)                                                                                        \
            return __;                                                                                                 \
    }                                                                                                                  \
    while (0)
#define CHECK_ARG(VAL)                                                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        if (!(VAL))                                                                                                    \
            return ESP_ERR_INVALID_ARG;                                                                                \
    }                                                                                                                  \
    while (0)

static void isr_disable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_DISABLE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_DISABLE);
}

static void isr_enable(wiegand_reader_t *reader)
{
    gpio_set_intr_type(reader->gpio_d0, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(reader->gpio_d1, GPIO_INTR_NEGEDGE);
}

#if HELPER_TARGET_IS_ESP32
static void IRAM_ATTR isr_handler(void *arg)
#else
static void isr_handler(void *arg)
#endif
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;
    if (!reader->enabled)
        return;

    int d0 = gpio_get_level(reader->gpio_d0);
    int d1 = gpio_get_level(reader->gpio_d1);

    // ignore equal
    if (d0 == d1)
        return;
    // overflow
    if (reader->bits >= reader->size * 8)
        return;

    esp_timer_stop(reader->timer);

    uint8_t value;
    if (reader->bit_order == WIEGAND_MSB_FIRST)
        value = (d0 ? 0x80 : 0) >> (reader->bits % 8);
    else
        value = (d0 ? 1 : 0) << (reader->bits % 8);

    if (reader->byte_order == WIEGAND_MSB_FIRST)
        reader->buf[reader->size - reader->bits / 8 - 1] |= value;
    else
        reader->buf[reader->bits / 8] |= value;

    reader->bits++;

    esp_timer_start_once(reader->timer, TIMER_INTERVAL_US);
}

static void timer_handler(void *arg)
{
    wiegand_reader_t *reader = (wiegand_reader_t *)arg;

    ESP_LOGD(TAG, "Got %d bits of data", reader->bits);

    wiegand_reader_disable(reader);

    if (reader->callback)
        reader->callback(reader);

    wiegand_reader_enable(reader);

    isr_enable(reader);
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t wiegand_reader_init(wiegand_reader_t *reader[CONFIG_NUM_SUPPORTED_READERS],
    gpio_num_t gpio_d0[CONFIG_NUM_SUPPORTED_READERS], gpio_num_t gpio_d1[CONFIG_NUM_SUPPORTED_READERS],
    bool internal_pullups, size_t buf_size, wiegand_callback_t callback, wiegand_order_t bit_order,
    wiegand_order_t byte_order, uint8_t num_wie_rd)
{
    uint8_t i;

    for (i = 0; i < num_wie_rd; i++)
    {
        CHECK_ARG(reader[i] && buf_size && callback);
				ESP_LOGI(TAG, "CHECK_ARG(reader[%d] && buf_size && callback)", i);
    }

    esp_err_t res = gpio_install_isr_service(0);
    if (res != ESP_OK && res != ESP_ERR_INVALID_STATE)
        return res;

    for (i = 0; i < num_wie_rd; i++)
    {
        memset(reader[i], 0, sizeof(wiegand_reader_t));
        reader[i]->gpio_d0 = gpio_d0[i];
        reader[i]->gpio_d1 = gpio_d1[i];
        reader[i]->size = buf_size;
        reader[i]->buf = calloc(buf_size, 1);
        reader[i]->bit_order = bit_order;
        reader[i]->byte_order = byte_order;
        reader[i]->callback = callback;

        esp_timer_create_args_t timer_args
            = { .name = TAG, .arg = reader[i], .callback = timer_handler, .dispatch_method = ESP_TIMER_TASK };
        CHECK(esp_timer_create(&timer_args, &reader[i]->timer));

        CHECK(gpio_set_direction(gpio_d0[i], GPIO_MODE_INPUT));
        CHECK(gpio_set_direction(gpio_d1[i], GPIO_MODE_INPUT));
        CHECK(gpio_set_pull_mode(gpio_d0[i], internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
        CHECK(gpio_set_pull_mode(gpio_d1[i], internal_pullups ? GPIO_PULLUP_ONLY : GPIO_FLOATING));
        isr_disable(reader[i]);
        CHECK(gpio_isr_handler_add(gpio_d0[i], isr_handler, reader[i]));
        CHECK(gpio_isr_handler_add(gpio_d1[i], isr_handler, reader[i]));
        isr_enable(reader[i]);
        reader[i]->enabled = true;
        reader[i]->rd_idx = i;

        ESP_LOGI(TAG, "Reader[%d] initialized on D0=%d, D1=%d", i, gpio_d0[i], gpio_d1[i]);
    }

    return ESP_OK;
}

esp_err_t wiegand_reader_disable(wiegand_reader_t *reader)
{
    CHECK_ARG(reader);

    isr_disable(reader);
    esp_timer_stop(reader->timer);
    reader->enabled = false;

    ESP_LOGD(TAG, "Reader on D0=%d, D1=%d disabled", reader->gpio_d0, reader->gpio_d1);

    return ESP_OK;
}

esp_err_t wiegand_reader_enable(wiegand_reader_t *reader)
{
    CHECK_ARG(reader);

    reader->bits = 0;
    memset(reader->buf, 0, reader->size);

    isr_enable(reader);
    reader->enabled = true;

    ESP_LOGD(TAG, "Reader on D0=%d, D1=%d enabled", reader->gpio_d0, reader->gpio_d1);

    return ESP_OK;
}

esp_err_t wiegand_reader_done(wiegand_reader_t *reader)
{
    CHECK_ARG(reader && reader->buf);

    isr_disable(reader);
    CHECK(gpio_isr_handler_remove(reader->gpio_d0));
    CHECK(gpio_isr_handler_remove(reader->gpio_d1));
    esp_timer_stop(reader->timer);
    CHECK(esp_timer_delete(reader->timer));
    free(reader->buf);

    ESP_LOGD(TAG, "Reader removed");

    return ESP_OK;
}
