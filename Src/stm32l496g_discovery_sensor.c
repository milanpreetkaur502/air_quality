/**
  ******************************************************************************
  * File Name          :  stm32l496g_discovery_sensor.c
  * Description        : This file provides code for the configuration
  *                      of the sensor instances.
  ******************************************************************************
  *
  * COPYRIGHT 2021 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l496g_discovery_sensor.h"
#include "main.h"
#include <stdio.h>

#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "stm32l4xx_hal.h"
#include "stm32l496g_discovery.h"

#include "math.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
  int8_t sign; /* 0 means positive, 1 means negative*/
  uint32_t  out_int;
  uint32_t  out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private variables ---------------------------------------------------------*/
static volatile uint8_t PushButtonDetected = 0;
static char dataOut[MAX_BUF_SIZE];


/* Private function prototypes -----------------------------------------------*/
static void Accelero_Sensor_Handler(uint32_t Instance);
static void Gyro_Sensor_Handler(uint32_t Instance);
static void Magneto_Sensor_Handler(uint32_t Instance);
static void Temp_Sensor_Handler(uint32_t Instance);
static void Hum_Sensor_Handler(uint32_t Instance);
static void Press_Sensor_Handler(uint32_t Instance);
void MX_IKS01A3_Init(void);
static void MX_IKS01A3_Process(void);

float convertCoord( float x, uint8_t sign );

extern IKS01A3_MOTION_SENSOR_Axes_t  acc_value;
extern IKS01A3_MOTION_SENSOR_Axes_t  gyr_value;
extern IKS01A3_MOTION_SENSOR_Axes_t  mag_value;
extern float_t temperature_value;
extern float_t humidity_value;
extern float_t pressure_value;

extern GNSSParser_Data_t GNSSParser_Data;
float teseo_latitude,teseo_longitude;

int32_t BSP_GetTick(void)
{
  uint32_t ret;
  ret = HAL_GetTick();
  return (int32_t)ret;
}

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */
  MX_IKS01A3_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A3_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */
  
  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the IKS01A3 sensors
  * @retval None
  */
void MX_IKS01A3_Init(void)
{
  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_HUMIDITY);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE);

  IKS01A3_ENV_SENSOR_Init(IKS01A3_STTS751_0, ENV_TEMPERATURE);

  if(IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0, MOTION_ACCELERO | MOTION_GYRO)!= 0)
  {
    printf("Failed to init %s\n", "IKS01A3_LSM6DSO_0");
    Error_Handler();
  }

  if(IKS01A3_MOTION_SENSOR_Init(IKS01A3_LIS2MDL_0, MOTION_MAGNETO)!= 0)
  {
    printf("Failed to init %s\n", "IKS01A3_LIS2MDL_0");
    Error_Handler();
  }
}


/**
  * @brief  Process of the IKS01A3 sensors reading
  * @retval None
  */
void MX_IKS01A3_Process(void)
{
  Accelero_Sensor_Handler(IKS01A3_LSM6DSO_0);
  Gyro_Sensor_Handler(IKS01A3_LSM6DSO_0);
  Magneto_Sensor_Handler(IKS01A3_LIS2MDL_0);
  
  Hum_Sensor_Handler(IKS01A3_HTS221_0);
  Temp_Sensor_Handler(IKS01A3_STTS751_0);
  Press_Sensor_Handler(IKS01A3_LPS22HH_0);

  HAL_Delay( 1000 );
}


/**
  * @brief  Handles the accelerometer axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Accelero_Sensor_Handler(uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t acceleration;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    acc_value.x = acceleration.x; acc_value.y = acceleration.y; acc_value.z = acceleration.z;
  }


}

/**
  * @brief  Handles the gyroscope axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Gyro_Sensor_Handler(uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t angular_velocity;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR[%d]: Error\r\n", (int)Instance);
  }
  else
  {
   gyr_value.x = angular_velocity.x; gyr_value.y = angular_velocity.y; gyr_value.z = angular_velocity.z;
  }

}

/**
  * @brief  Handles the magneto axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Magneto_Sensor_Handler(uint32_t Instance)
{
  IKS01A3_MOTION_SENSOR_Axes_t magnetic_field;

  if (IKS01A3_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nMAG[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    mag_value.x = magnetic_field.x; mag_value.y = magnetic_field.y; mag_value.z = magnetic_field.z;
  }


}

/**
  * @brief  Handles the temperature data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Temp_Sensor_Handler(uint32_t Instance)
{
  float temperature;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    temperature_value = temperature;
  }


}

/**
  * @brief  Handles the pressure sensor data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Press_Sensor_Handler(uint32_t Instance)
{
  float pressure;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    pressure_value = pressure;
  }

}

/**
  * @brief  Handles the humidity data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Hum_Sensor_Handler(uint32_t Instance)
{
  float humidity;

  if (IKS01A3_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity))
  {
    snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: Error\r\n", (int)Instance);
  }
  else
  {
    humidity_value = humidity;
  }
}


/**
  * @brief  Handles reading of GNSS coordinates
  * @param  teseo_latitude
  * @param  teseo_longitude
  * @retval None
  */
void GNSS_Sensor_Handler(float *teseo_latitude, float *teseo_longitude)
{
  printf("Reading GNSS... ");
  
  if (GNSSParser_Data.gpgga_data.valid != (uint8_t)VALID) 
  {    
    printf("Position not get (using last known).\r\n"); 
  }
  else
  {
    printf("Position just get.\r\n");
  }
  
  *teseo_latitude  = convertCoord( GNSSParser_Data.gpgga_data.xyz.lat, GNSSParser_Data.gpgga_data.xyz.ns=='N' ? 0 : 1 );
  *teseo_longitude = convertCoord( GNSSParser_Data.gpgga_data.xyz.lon, GNSSParser_Data.gpgga_data.xyz.ew=='E' ? 0 : 1 );
  return;
}

/**
  * @brief  Convert latitude/longitude coordinate from sexagesimal to decimal format
  * @param  float x coordinate
  * @param  uint8_t sign 1 for negative 0 for positive
  * @retval coordinate in decimal format
  */
float convertCoord( float x, uint8_t sign )
{
  int degrees;
  float minutes;
  float ret;

  degrees = (int)(x / 100.0F);
  minutes = x - degrees*100.0F;
  ret = degrees + minutes / 60.0F;
  if (sign==1)
    ret = -ret;
  
  return ret;
}

    
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
