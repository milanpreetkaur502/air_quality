/**
  ******************************************************************************
  * @file    stm32l496g_discovery_patch.c
  * @author  MCD Application Team
  * @brief   This file provides a set of firmware functions to interface I2C1.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32l496g_discovery.h"
#include "stm32l496g_discovery_patch.h"
#include "stm32l496g_discovery_io.h"

/** @addtogroup BSP
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY STM32L496G-DISCOVERY
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY_Common STM32L496G-DISCOVERY Common
  * @{
  */

/** @defgroup STM32L496G_DISCOVERY_Private_TypesDefinitions Private Types Definitions
  * @brief This file provides firmware functions to manage Leds, push-buttons,
  *        COM ports, SD card on SPI and temperature sensor (TS751) available on
  *        STM32L496G-DISCOVERY discoveryuation board from STMicroelectronics.
  * @{
  */

/**
  * @}
  */

/** @defgroup STM32L496G_DISCOVERY_Private_Defines Private Defines
  * @{
  */




/** @defgroup STM32L496G_DISCOVERY_Private_Macros Private Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup STM32L496G_DISCOVERY_Exported_Variables Exported Variables
  * @{
  */


/**
 * @brief BUS variables
 */
#if defined(HAL_I2C_MODULE_ENABLED)
extern uint32_t I2c1Timeout;  /*<! Value of Timeout when I2C1 communication fails */
static I2C_HandleTypeDef I2c1Handle = {0};



/** @defgroup STM32L496G_DISCOVERY_Private_FunctionPrototypes Private Functions
  * @{
  */

/* BUS IO driver over I2C Peripheral */
/*******************************************************************************
                            BUS OPERATIONS OVER I2C
*******************************************************************************/
/**
  * @brief  Initialize I2C HAL
  * @retval BSP status
  */
int32_t BSP_I2C1_Init(void)
{
  int32_t ret = 0;
  if (HAL_I2C_GetState(&I2c1Handle) == HAL_I2C_STATE_RESET)
  {
    I2c1Handle.Instance              = DISCOVERY_I2C1;
    I2c1Handle.Init.Timing           = DISCOVERY_I2C1_TIMING;
    I2c1Handle.Init.OwnAddress1      = 0x00;
    I2c1Handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    I2c1Handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    I2c1Handle.Init.OwnAddress2      = 0x00;
    I2c1Handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    I2c1Handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    /* Init the I2C */
    I2C1_MspInit(&I2c1Handle);
    HAL_I2C_Init(&I2c1Handle);
  }
  return ret;
}

/**
  * @brief Discovery I2C1 MSP Initialization
  * @param hi2c: I2C1 handle
  * @retval None
  */
static void I2C1_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  if (hi2c->Instance == DISCOVERY_I2C1)
  {
    /*##-1- Configure the Discovery I2C1 clock source. The clock is derived from the SYSCLK #*/
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    /*##-2- Configure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    GPIO_InitStructure.Pin       = DISCOVERY_I2C1_SCL_PIN;
    GPIO_InitStructure.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull      = /*GPIO_NOPULL*/ GPIO_PULLUP;
    GPIO_InitStructure.Speed     = /*GPIO_SPEED_MEDIUM*/ GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Alternate = DISCOVERY_I2C1_SCL_SDA_AF;
    HAL_GPIO_Init(DISCOVERY_I2C1_SCL_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.Pin       = DISCOVERY_I2C1_SDA_PIN;
    HAL_GPIO_Init(DISCOVERY_I2C1_SDA_GPIO_PORT, &GPIO_InitStructure);

    /*##-3- Configure the Discovery I2C1 peripheral #############################*/
    /* Enable Discovery_I2C1 clock */
    DISCOVERY_I2C1_CLK_ENABLE();

    /* Force and release the I2C Peripheral Clock Reset */
    DISCOVERY_I2C1_FORCE_RESET();
    DISCOVERY_I2C1_RELEASE_RESET();

    /* Enable and set Discovery I2C1 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C1_EV_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C1_EV_IRQn);

    /* Enable and set Discovery I2C1 Interrupt to the highest priority */
    HAL_NVIC_SetPriority(DISCOVERY_I2C1_ER_IRQn, 0x01, 0);
    HAL_NVIC_EnableIRQ(DISCOVERY_I2C1_ER_IRQn);
  }
}

/**
  * @brief  DeInitialize I2C HAL.
  * @retval BSP status
  */
int32_t BSP_I2C1_DeInit(void)
{
  if (HAL_I2C_GetState(&I2c1Handle) != HAL_I2C_STATE_RESET)
  {
    /* DeInit the I2C */
    HAL_I2C_DeInit(&I2c1Handle);
    I2C1_MspDeInit(&I2c1Handle);
  }
  return 0;
}

/**
  * @brief Discovery I2C1 MSP DeInitialization
  * @param hi2c: I2C1 handle
  * @retval None
  */
static void I2C1_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == DISCOVERY_I2C1)
  {
    /*##-1- Unconfigure the GPIOs ################################################*/
    /* Enable GPIO clock */
    DISCOVERY_I2C1_SDA_GPIO_CLK_ENABLE();
    DISCOVERY_I2C1_SCL_GPIO_CLK_ENABLE();

    /* Configure I2C Rx/Tx as alternate function  */
    HAL_GPIO_DeInit(DISCOVERY_I2C1_SCL_GPIO_PORT, DISCOVERY_I2C1_SCL_PIN);
    HAL_GPIO_DeInit(DISCOVERY_I2C1_SDA_GPIO_PORT,  DISCOVERY_I2C1_SDA_PIN);

    /*##-2- Unconfigure the Discovery I2C1 peripheral ############################*/
    /* Force and release I2C Peripheral */
    DISCOVERY_I2C1_FORCE_RESET();
    DISCOVERY_I2C1_RELEASE_RESET();

    /* Disable Discovery I2C1 clock */
    DISCOVERY_I2C1_CLK_DISABLE();

    /* Disable Discovery I2C1 interrupts */
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(DISCOVERY_I2C1_ER_IRQn);
  }
}
/**
  * @brief  Check whether the I2C bus is ready.
  * @param DevAddr : I2C device address
  * @param Trials : Check trials number
  *	@retval BSP status
  */
int32_t BSP_I2C1_IsReady(uint16_t DevAddr, uint32_t Trials)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

  __disable_irq();

  status = HAL_I2C_IsDeviceReady(&I2c1Handle, DevAddr, Trials, 50);

  __enable_irq();

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
    HAL_Delay(200);
  }

  return value;
}

/**
  * @brief Discovery I2C1 error treatment function
  * @retval None
  */
static void I2C1_Error(void)
{
  //BSP_ErrorHandler();

  /* De-initialize the I2C communication BUS */
  HAL_I2C_DeInit(&I2c1Handle);

  /* Re- Initiaize the I2C communication BUS */
  BSP_I2C1_Init();
}

/**
  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write
  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP status
  */

int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  __disable_irq();

  status = HAL_I2C_Mem_Write(&I2c1Handle, DevAddr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, I2c1Timeout);

  __enable_irq();


  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
  }

  return status;
}

/**
  * @brief  Read a register of the device through BUS
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to read
  * @param  pData  Pointer to data buffer to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

  //__disable_irq();

  status = HAL_I2C_Mem_Read(&I2c1Handle, DevAddr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, I2c1Timeout);

  //__enable_irq();

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
    HAL_Delay(200);
  }

  return value;
}

/**

  * @brief  Write a value in a register of the device through BUS.
  * @param  DevAddr Device address on Bus.
  * @param  Reg    The target register address to write

  * @param  pData  Pointer to data buffer to write
  * @param  Length Data Length
  * @retval BSP statu
  */
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;

  __disable_irq();

  status = HAL_I2C_Mem_Write(&I2c1Handle, DevAddr, (uint16_t)Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, I2c1Timeout);

  __enable_irq();


  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
  }

  return status;
}

/**
  * @brief  Read registers through a bus (16 bits)
  * @param  DevAddr: Device address on BUS
  * @param  Reg: The target register address to read
  * @param  Length Data Length
  * @retval BSP status
  */
int32_t  BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0x0;

  __disable_irq();

  status = HAL_I2C_Mem_Read(&I2c1Handle, DevAddr, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, I2c1Timeout);

  __enable_irq();

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
    HAL_Delay(200);
  }

  return value;
}

/**
  * @brief  Send an amount width data through bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Send(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Master_Transmit(&I2c1Handle, DevAddr, pData, Length, I2c1Timeout);

  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
    HAL_Delay(200);
  }

  return status;
}

/**
  * @brief  Receive an amount of data through a bus (Simplex)
  * @param  DevAddr: Device address on Bus.
  * @param  pData: Data pointer
  * @param  Length: Data length
  * @retval BSP status
  */
int32_t BSP_I2C1_Recv(uint16_t DevAddr, uint8_t *pData, uint16_t Length) {
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Master_Receive(&I2c1Handle, DevAddr, pData, Length, I2c1Timeout);
  /* Check the communication status */
  if (status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    I2C1_Error();
    HAL_Delay(200);
  }
  return status;
}

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
/**
  * @brief Register Default BSP I2C1 Bus Msp Callbacks
  * @retval BSP status
  */
int32_t BSP_I2C1_RegisterDefaultMspCallbacks (void)
{
}

/**
  * @brief BSP I2C1 Bus Msp Callback registering
  * @param Callbacks     pointer to I2C1 MspInit/MspDeInit callback functions
  * @retval BSP status
  */
int32_t BSP_I2C1_RegisterMspCallbacks (BSP_I2C_Cb_t *Callbacks)
{
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#endif /* HAL_I2C_MODULE_ENABLED */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
