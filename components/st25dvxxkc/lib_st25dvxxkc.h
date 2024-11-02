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
 * @file lib_st25dvxxkc.h
 * @defgroup lib_st25dvxxkc lib_st25dvxxkc
 * @{
 *
 * ESP-IDF driver for st25dvxxkc
 *
 * Copyright (c) 2024 WH Wong
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __LIB_ST25DVXXKC_H__
#define __LIB_ST25DVXXKC_H__

#include <i2cdev.h>
#include <stdbool.h>
#include <time.h>
#include <esp_err.h>

#include "st25dvxxkc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ST25DVxxKC_I2C_ADDR ST25DVXXKC_ADDR_DATA_I2C

/************************/
/* byte-swapping macros */
/************************/
#define BYTE_SWAP_16(x)	((((uint16_t)(x)&0xff00)>>8) | (((uint16_t)(x)&0x00ff)<<8))
#define BYTE_SWAP_32(x)	((((uint32_t)(x)&0xff000000)>>24) | (((uint32_t)(x)&0x00ff0000)>>8) | (((uint32_t)(x)&0x0000ff00)<<8) | (((uint32_t)(x)&0x000000ff)<<24))
#define BYTE_SWAP_64(x)	((((uint64_t)(x) & 0xff00000000000000ULL) >> 56) | \
						( ((uint64_t)(x) & 0x00ff000000000000ULL) >> 40) | \
						( ((uint64_t)(x) & 0x0000ff0000000000ULL) >> 24) | \
						( ((uint64_t)(x) & 0x000000ff00000000ULL) >> 8)  | \
						( ((uint64_t)(x) & 0x00000000ff000000ULL) << 8)  | \
						( ((uint64_t)(x) & 0x0000000000ff0000ULL) << 24) | \
						( ((uint64_t)(x) & 0x000000000000ff00ULL) << 40) | \
						( ((uint64_t)(x) & 0x00000000000000ffULL) << 56))

/* auto-detect integer size */
#define BYTE_SWAP_INT(x)	(sizeof(x)==2 ? BYTE_SWAP_16(x) : sizeof(x)==4 ? BYTE_SWAP_32(x) : sizeof(x)==8 ? BYTE_SWAP_64(x) : (x))


esp_err_t st25dvxxkc_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t st25dvxxkc_free_desc(i2c_dev_t *dev);

int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials);
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __LIB_ST25DVXXKC_H__ */
