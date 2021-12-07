/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   MSP Initialization and de-Initialization.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config_bus.h"

/**
  * Initializes the Global MSP.
  */

 void Periph_Config(void)
 {
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                                       |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                                       |RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection  = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection  = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection    = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection     = RCC_RNGCLKSOURCE_MSI;

  PeriphClkInit.RTCClockSelection     = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

   __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
      /* Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
  }
}

void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();
  }
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    __HAL_RCC_RTC_ENABLE();
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  }
}

/**
 * @brief  This function Initializes the I2C
 * @param  None
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==GNSS_I2C_INSTANCE)
  {
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GNSS_I2C_SCL|GNSS_I2C_SDA;
    GPIO_InitStruct.Mode = GNSS_I2C_PIN_MODE;
    GPIO_InitStruct.Pull = GNSS_I2C_PULL;
    GPIO_InitStruct.Speed = GNSS_I2C_SPEED;
    GPIO_InitStruct.Alternate = GNSS_I2C_ALTERNATE;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    GNSS_I2C_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(GNSS_I2C_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GNSS_I2C_EV_IRQn);
    HAL_NVIC_SetPriority(GNSS_I2C_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GNSS_I2C_ER_IRQn);
  }

}

/**
 * @brief  This function De-Initializes the I2C
 * @param  None
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==GNSS_I2C_INSTANCE)
  {
    /* Peripheral clock disable */
    GNSS_I2C_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GNSS_I2C_PORT, GNSS_I2C_SCL|GNSS_I2C_SDA);

    /* I2C1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(GNSS_I2C_EV_IRQn);
    HAL_NVIC_DisableIRQ(GNSS_I2C_ER_IRQn);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
