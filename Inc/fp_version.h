/**
  ******************************************************************************
  * @file    fp_version.h
  * @author  SRA Team
  * @brief   STM32 AWS Function Pack FW version definition
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

/**
   \mainpage STM32Cube function pack for asset tracking with LTE connectivity, GNSS and MEMS sensors
   *
   * Introduction
   * ------------
   *
   * This user manual describes the STM32Cube Function Pack software for LTE Asset Tracking
   *
   * The Package is based on a number of other software Packages covering a specific category:
   * - STM32L496G Discovery BSP
   * - STM32 Sensor and GNSS BSP
   * - FreeRTOS Middleware
   * - MbedTLS Middleware
   * - Amazon AWS IoT Device Embedded-C SDK Middleware
   *
   * Using the Pack
   * ------------
   * To use the Pack go to <code>Projects\\32L496GDISCOVERY\\Applications\\Cloud\\AssetTracker</code> folder.
   * Select any of of the tool chains from Keil (MDK-ARM) or IAR (EWARM) or STM32CubeIDE.
   * Build the project and run it.
   * Connect a Hyper Terminal to Platform Board to see the results.
   *
   * Examples
   * --------
   *
   * The Package ships with an application example that connects to ST's Asset Tracking Dashboard to publish sensors data. 
   * This Example demonstrate how to Send Data to AWS cloud and how to present them to user..
   *
   * Tool chain Support
   * ------------
   *
   * The Package has been developed and tested with :
   *   - IAR Embedded Workbench for ARM version 8.50.4
   *   - MDK-ARM version 5.31.0
   *   - STM32CubeIDE version 1.5.1
   *
   * Building the Package
   * ------------
   *
   * The Package contains a project files to build Project.
   * - On IAR Embedded Workbench Tool chain the project file is in <code>Projects\\32L496GDISCOVERY\\Applications\\Cloud\\AssetTracker\\EWARM</code> folder : 
   * Project.eww
   * - On MDK-ARM Tool chain the project file is in <code>Projects\\32L496GDISCOVERY\\Applications\\Cloud\\AssetTracker\\MDK-ARM</code> folder : 
   * Project.uvprojx
   * - On STM32CubeIDE Tool chain the project file is in <code>Projects\\32L496GDISCOVERY\\Applications\\Cloud\\AssetTracker\\STM32CubeIDE</code> folder : 
   * .project
   *
   *
   *
   * Pre-processor Macros
   * ------------
   *
   * Each Package project have different pre-processor macros.
   *
   *
   * - IOT_INFO :
   * Define macro IOT_INFO, If you want to receive print messages on UART from AWS IoT part of module of generic Information type
   * - IOT_ERROR :
   * Define macro IOT_ERROR, If you want to receive print messages on UART from AWS IoT part of module of Error type
   * - IOT_WARN :
   * Define macro IOT_WARN, If you want to receive print messages on UART from AWS IoT part of module of Warning type
   * - IOT_DEBUG :
   * Define macro IOT_DEBUG, If you want to receive print messages on UART from AWS IoT part of module of Extended Information type
   * - IOT_TRACE :
   * Define macro IOT_TRACE, If you want to receive print messages on UART from AWS IoT part of module of all Trace type
   *
   *
   * <hr>
   * STM32 Cube Function Pack for LTE Asset Tracking 
   * ------------------------------------
   * 
   * The following files relevant to Package are present in the function pack directories:
   * |File/Folder                   |Content                                                                   |
   * |------------------------------|--------------------------------------------------------------------------|
   * |\b Documentation              | This documentation                                                       |
   * |\b Drivers                    | BSP Drivers                                                              |
   * |\b Middlewares                | GNSS, Cellular, FreeRTOS, LwIP, MbedTLS, JSMN and AWS IOT source code    |
   * |\b Projects                   | Source files for rebuilding the Package                                  |
   * 
   * <hr>
   * Revision History of STM32 Cube Function Pack for LTE Asset Tracking
   * --------------------------------------------------------
   * Version 1.1.0
   * Version 1.0.0 
   *
   * Copyright Notice
   * ----------------
   *
   * COPYRIGHT(c) 2021 STMicroelectronics.
   */
  
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FP_VERSION_H
#define FP_VERSION_H

#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 1
#define FW_VERSION_PATCH 0
#define FW_VERSION_DATE "22-February-2021 11:30:00 AM"

#define FW_VERSION_NAME   "FP-ATR-LTE1"

#endif /* FP_VERSION_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
