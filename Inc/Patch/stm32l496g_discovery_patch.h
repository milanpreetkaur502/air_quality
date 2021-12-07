/**
  ******************************************************************************
  * @file    stm32l496g_discovery_patch.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for STM32L496G_DISCOVERY's I2C1 
             Interface routines and redefinition of GPIO allocation defines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32L496G_DISCOVERY_EX_H
#define __STM32L496G_DISCOVERY_EX_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Define for STM32L496G_DISCOVERY board
  */



/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l496g_discovery.h"
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L496G_DISCOVERY
  * @{
  */

/** @addtogroup STM32L496G_DISCOVERY_Common
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY_Exported_Types Exported Types
  * @{
  */







/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_BUS  BUS Constants
  * @{
  */

#if defined(HAL_I2C_MODULE_ENABLED)
/*##################### I2C1 ###################################*/
/* User can use this section to tailor I2C1 instance used and associated
   resources */
#if defined (SENSOR) /* When I2C1 is used to connect Senors on X-NUCLEO-IKS01A3*/


#undef DISCOVERY_I2C1_SCL_GPIO_PORT
#undef DISCOVERY_I2C1_SDA_GPIO_PORT
#undef DISCOVERY_I2C1_SCL_PIN
#undef DISCOVERY_I2C1_SDA_PIN
#undef DISCOVERY_I2C1_SCL_SDA_AF
#undef DISCOVERY_I2C1
#undef DISCOVERY_I2C1_CLK_ENABLE
#undef DISCOVERY_I2C1_CLK_DISABLE
#undef DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE
#undef DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE
#undef DISCOVERY_I2C1_SDA_GPIO_CLK_DISABLE
#undef DISCOVERY_I2C1_SCL_GPIO_CLK_DISABLE
#undef DISCOVERY_I2C1_FORCE_RESET
#undef DISCOVERY_I2C1_RELEASE_RESET
#undef DISCOVERY_I2C1_EV_IRQn
#undef DISCOVERY_I2C1_EV_IRQHandler
#undef DISCOVERY_I2C1_ER_IRQn
#undef DISCOVERY_I2C1_ER_IRQHandler
#ifdef DISCOVERY_I2C1_TIMING
#undef DISCOVERY_I2C1_TIMING
#endif /* DISCOVERY_I2C1_TIMING */
#ifdef BSP_I2C_SPEED
#undef BSP_I2C_SPEED
#endif /* BSP_I2C_SPEED */
#undef DISCOVERY_I2C1_TIMEOUT_MAX

/* Definition for I2C1 Pins */
#define DISCOVERY_I2C1_SCL_GPIO_PORT            GPIOB
#define DISCOVERY_I2C1_SDA_GPIO_PORT            GPIOB
#define DISCOVERY_I2C1_SCL_PIN                  GPIO_PIN_8
#define DISCOVERY_I2C1_SDA_PIN                  GPIO_PIN_7

#define DISCOVERY_I2C1_SCL_SDA_AF               GPIO_AF4_I2C1

/* Definition for I2C1 clock resources */
#define DISCOVERY_I2C1                          I2C1
#define DISCOVERY_I2C1_CLK_ENABLE()             __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2C1_CLK_DISABLE()            __HAL_RCC_I2C1_CLK_DISABLE()
#define DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define DISCOVERY_I2C1_SDA_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C1_SCL_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()
#define DISCOVERY_I2C1_FORCE_RESET()            __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2C1_RELEASE_RESET()          __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2C1's NVIC */
#define DISCOVERY_I2C1_EV_IRQn                  I2C1_EV_IRQn
#define DISCOVERY_I2C1_EV_IRQHandler            I2C1_EV_IRQHandler
#define DISCOVERY_I2C1_ER_IRQn                  I2C1_ER_IRQn
#define DISCOVERY_I2C1_ER_IRQHandler            I2C1_ER_IRQHandler

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated in case of the I2C Clock source is the SYSCLK = 80 MHz */
/* Set 0x90112626 value to reach 100 KHz speed (Rise time = 640ns, Fall time = 20ns) */
#ifndef DISCOVERY_I2C1_TIMING
#define DISCOVERY_I2C1_TIMING                  0x90D00e28/* 0x90112626*/
#endif /* DISCOVERY_I2C1_TIMING */

/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   stm324xg_discovery.h file). It can be used in parallel by other modules. */
#ifndef BSP_I2C_SPEED
#define BSP_I2C_SPEED                              100000
#endif /* BSP_I2C_SPEED */

/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define DISCOVERY_I2C1_TIMEOUT_MAX              3000
#endif /* SENSOR */


/**************************** Bus functions ************************************/
/* I2C1 bus function */

/* BUS IO driver over I2C Peripheral */
int32_t BSP_I2C1_Init(void);
static void I2C1_MspInit(I2C_HandleTypeDef *hi2c);
int32_t BSP_I2C1_DeInit(void);
static void I2C1_MspDeInit(I2C_HandleTypeDef *hi2c);
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials);
static void I2C1_Error(void);
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_WriteReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg16(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_SendRecv(uint16_t DevAddr, uint8_t *pTxdata, uint8_t *pRxdata, uint16_t Length);

int32_t BSP_GetTick(void);

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
typedef struct
{
  pI2C_CallbackTypeDef  pMspInitCb;
  pI2C_CallbackTypeDef  pMspDeInitCb;
}BSP_I2C_Cb_t;

int32_t BSP_I2C1_RegisterDefaultMspCallbacks (void);
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks);
#endif /* (USE_HAL_I2C_REGISTER_CALLBACKS == 1) */

#endif/* HAL_I2C_MODULE_ENABLED */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32L496G_DISCOVERY_EX_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
