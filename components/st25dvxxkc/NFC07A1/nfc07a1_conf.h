/**
 ******************************************************************************
 * @file    nfc07a1_conf_template.h
 * @author  MMY Application Team
 * @brief   This file contains definitions for the NFC4 components bus interfaces
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef _NFC07A1_CONF_H_
#define _NFC07A1_CONF_H_

#include "lib_st25dvxxkc.h"

/* Bus IO */
#define NFC07A1_I2C_Init       BSP_I2C1_Init
#define NFC07A1_I2C_DeInit     BSP_I2C1_DeInit
#define NFC07A1_I2C_ReadReg16  BSP_I2C1_ReadReg16
#define NFC07A1_I2C_WriteReg16 BSP_I2C1_WriteReg16
#define NFC07A1_I2C_Recv       BSP_I2C1_Recv
#define NFC07A1_I2C_IsReady    BSP_I2C1_IsReady
// #define NFC07A1_GetTick        xTaskGetTickCount

#define HAL_GetTick xTaskGetTickCount

#define NFC07A1_NFCTAG_INSTANCE         (0)

/**
 * @}
 */

#endif /* _NFC07A1_CONF_H_ */
