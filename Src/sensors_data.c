/**
  ******************************************************************************
  * @file    sensors_data.c
  * @author  MCD Application Team
  * @brief   Prepare data of X-Nucleo-IKS01A3 sensors.
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "sensors_data.h"

#include "stm32l4xx_hal.h"
#if defined (STM32L475xx)
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_env_sensors.h"
#include "stm32l475e_iot01_motion_sensors.h"
#include "vl53l0x_proximity.h"
#endif
#include "msg.h"
#if defined (STM32L496xx)
#include "iks01a3_motion_sensors.h"
#include "iks01a3_env_sensors.h"
#include "stm32l496g_discovery_sensor.h"
#include "stm32l496g_discovery.h"
#endif
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define INSTANCE_TEMPERATURE_HUMIDITY 0
#define INSTANCE_TEMPERATURE_PRESSURE 1
#define INSTANCE_GYROSCOPE_ACCELEROMETER 0
#define INSTANCE_MAGNETOMETER 1
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/

/**
  * @brief  init_sensors
  * @param  none
  * @retval 0  in case of success
  *         <0 BSP error code in case of failure
  */
int init_sensors(void)
{
  int32_t ret = 0;

  msg_info("Initializing sensors... ");
  
#if defined (STM32L475xx)

  ret = BSP_ENV_SENSOR_Init(INSTANCE_TEMPERATURE_HUMIDITY, ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Init(ENV_HUMIDITY) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_ENV_SENSOR_Enable(INSTANCE_TEMPERATURE_HUMIDITY, ENV_HUMIDITY);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Enable(ENV_HUMIDITY) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_ENV_SENSOR_Init(INSTANCE_TEMPERATURE_HUMIDITY, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Init(ENV_TEMPERATURE) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_ENV_SENSOR_Enable(INSTANCE_TEMPERATURE_HUMIDITY, ENV_TEMPERATURE);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Enable(ENV_TEMPERATURE) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_ENV_SENSOR_Init(INSTANCE_TEMPERATURE_PRESSURE, ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Init(ENV_PRESSURE) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_ENV_SENSOR_Enable(INSTANCE_TEMPERATURE_PRESSURE, ENV_PRESSURE);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_ENV_SENSOR_Enable(ENV_PRESSURE) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Init(INSTANCE_MAGNETOMETER, MOTION_MAGNETO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Init(MOTION_MAGNETO) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Enable(INSTANCE_MAGNETOMETER, MOTION_MAGNETO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Enable(MOTION_MAGNETO) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Init(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_GYRO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Init(MOTION_GYRO) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Enable(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_GYRO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Enable(MOTION_GYRO) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Init(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Init(MOTION_ACCELERO) returns %ld\n", ret);
    goto error;
  }

  ret = BSP_MOTION_SENSOR_Enable(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO);
  if (ret != BSP_ERROR_NONE)
  {
    msg_error("BSP_MOTION_SENSOR_Enable(MOTION_ACCELERO) returns %ld\n", ret);
    goto error;
  }

  VL53L0X_PROXIMITY_Init();

#elif defined (STM32L496xx)

  MX_MEMS_Init();

#endif /* STM32L496xx */

  msg_info("OK.\r\n");

  return ret;
}

/**
  * @brief  fill the buffer with the sensor values
  * @param  none
  * @param Buffer is the char pointer for the buffer to be filled
  * @param Size size of the above buffer
  * @retval 0 in case of success
  *         -1 in case of failure
  */
#ifdef STM32L496xx
  IKS01A3_MOTION_SENSOR_Axes_t  acc_value = {0, 0, 0};
  IKS01A3_MOTION_SENSOR_Axes_t  gyr_value = {0, 0, 0};
  IKS01A3_MOTION_SENSOR_Axes_t  mag_value = {0, 0, 0};
  float_t    temperature_value = 0.0;
  float_t    humidity_value = 0.0;
  float_t    pressure_value = 0.0;
  float_t    longitude = 0.0;
  float_t    latitude = 0.0;

  extern int use_stored_coordinates;
  extern float stored_latitude, stored_longitude;
#endif

int PrepareSensorsData(char * Buffer, int Size, char * deviceID)
{
  int rc = -1;
    
  msg_info("Reading sensors... ");
  
#if defined (STM32L475xx)
  float_t    temperature_value = 0.0;
  float_t    humidity_value = 0.0;
  float_t    pressure_value = 0.0;
  BSP_MOTION_SENSOR_Axes_t  acc_value = {0, 0, 0};
  BSP_MOTION_SENSOR_Axes_t  gyr_value = {0, 0, 0};
  BSP_MOTION_SENSOR_Axes_t  mag_value = {0, 0, 0};
  uint16_t proximity_value = 0;

  BSP_ENV_SENSOR_GetValue(INSTANCE_TEMPERATURE_HUMIDITY, ENV_TEMPERATURE, &temperature_value);
  BSP_ENV_SENSOR_GetValue(INSTANCE_TEMPERATURE_HUMIDITY, ENV_HUMIDITY, &humidity_value);
  BSP_ENV_SENSOR_GetValue(INSTANCE_TEMPERATURE_PRESSURE, ENV_PRESSURE, &pressure_value);
  proximity_value = VL53L0X_PROXIMITY_GetDistance();
  BSP_MOTION_SENSOR_GetAxes(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_ACCELERO, &acc_value);
  BSP_MOTION_SENSOR_GetAxes(INSTANCE_GYROSCOPE_ACCELEROMETER, MOTION_GYRO, &gyr_value);
  BSP_MOTION_SENSOR_GetAxes(INSTANCE_MAGNETOMETER, MOTION_MAGNETO, &mag_value);

#elif defined (STM32L496xx)

  //uint16_t proximity_value = 0;
  char * Buff = Buffer;
  int BuffSize = Size;
  int snprintfreturn = -1;

  MX_MEMS_Process();

#endif /* STM32L496xx   */

  msg_info("OK.\r\n");

  if ( !use_stored_coordinates )
  {
      // Use gnss coordinates
    GNSS_Sensor_Handler(&latitude, &longitude);
  }
  else
  {
    // Use stored coordinates instead
    latitude  = stored_latitude;
    longitude = stored_longitude;
  }
  
#ifdef AWS_IOT_DASHBOARD
  unsigned long long int timestamp = (unsigned long long int)time(NULL)*1000;
  
  snprintfreturn = snprintf( Buff, BuffSize, 
    "{ \"device_id\": \"%s\","
    "\"values\": ["
        "{ \"ts\": %lld, \"t\": \"tem\", \"v\": %.2f },"
        "{ \"ts\": %lld, \"t\": \"hum\", \"v\": %.2f },"
        "{ \"ts\": %lld, \"t\": \"pre\", \"v\": %.2f },"
        "{ \"ts\": %lld, \"t\": \"acc\", \"v\": { \"x\": %d, \"y\": %d, \"z\": %d } },"
        "{ \"ts\": %lld, \"t\": \"gyr\", \"v\": { \"x\": %d, \"y\": %d, \"z\": %d } },"
        "{ \"ts\": %lld, \"t\": \"mag\", \"v\": { \"x\": %d, \"y\": %d, \"z\": %d } },"
        "{ \"ts\": %lld, \"t\": \"gnss\", \"v\": { \"lat\": %f, \"lon\": %f, \"ele\": %f } }"
    "] }",
      deviceID, 
      timestamp, temperature_value, timestamp, humidity_value, timestamp, pressure_value, 
      timestamp, acc_value.x, acc_value.y, acc_value.z,
      timestamp, gyr_value.x, gyr_value.y, gyr_value.z,
      timestamp, mag_value.x, mag_value.y, mag_value.z,
      timestamp, latitude, longitude, 0.0);
#else
  if (deviceID != NULL)
  {
    snprintfreturn = snprintf( Buff, BuffSize, "{\"deviceId\":\"%s\","
             "\"temperature\": %.2f, \"humidity\": %.2f, \"pressure\": %.2f, \"proximity\": %d, "
             "\"acc_x\": %ld, \"acc_y\": %ld, \"acc_z\": %ld, "
             "\"gyr_x\": %ld, \"gyr_y\": %ld, \"gyr_z\": %ld, "
             "\"mag_x\": %ld, \"mag_y\": %ld, \"mag_z\": %ld, "
             "\"latitude\": %f, \"longitude\": %f "
             "}",
             deviceID,
             temperature_value, humidity_value, pressure_value, proximity_value,
             acc_value.x, acc_value.y, acc_value.z,
             gyr_value.x, gyr_value.y, gyr_value.z,
             mag_value.x, mag_value.y, mag_value.z,
             latitude, longitude);
  }
  else
  {
  snprintfreturn = snprintf( Buff, BuffSize, "{\n \"state\": {\n  \"reported\": {\n"
           "   \"temperature\": %.2f,\n   \"humidity\": %.2f,\n   \"pressure\": %.2f,\n   \"proximity\": %d,\n"
           "   \"acc_x\": %ld, \"acc_y\": %ld, \"acc_z\": %ld,\n"
           "   \"gyr_x\": %ld, \"gyr_y\": %ld, \"gyr_z\": %ld,\n"
           "   \"mag_x\": %ld, \"mag_y\": %ld, \"mag_z\": %ld,\n"
           "   \"latitude\": %f,\n   \"longitude\": %f\n"
           "  }\n }\n}",
           temperature_value, humidity_value, pressure_value, proximity_value,
           acc_value.x, acc_value.y, acc_value.z,
           gyr_value.x, gyr_value.y, gyr_value.z,
           mag_value.x, mag_value.y, mag_value.z,
           latitude, longitude);
  }
#endif

  /* Check total size to be less than buffer size
   * if the return is >=0 and <n, then
   * the entire string was successfully formatted; if the return is
   * >=n, the string was truncated (but there is still a null char
   * at the end of what was written); if the return is <0, there was
   * an error.
   */
  if (snprintfreturn >= 0 && snprintfreturn < Size)
  {
      rc = 0;
  }
  else if(snprintfreturn >= Size)
  {
      msg_error("Data Pack truncated\n");
  }
  else
  {
      msg_error("Data Pack Error\n");
  }

  return rc;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
