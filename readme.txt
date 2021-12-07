﻿/**
  @page LTE Asset Tracking application

  @verbatim
  ******************************************************************************
  * @file    readme.txt
  * @author  SRA Team
  * @brief   This application contains an example showing how to perform Asset Tracking by 
             connecting a P-L496G-CELL02 based microsystem with GNSS and MEMS sensors to 
	         ST's Asset Tracker Dashboard.
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0055, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0055
  *
  ******************************************************************************
  @endverbatim

@par Application Description


This application implements an Asset Tracking application for P-L496G-CELL02 kit with 
STM32L496G-Discovery and Quectel BG96 cellular LTE modem, equipped with X-NUCLEO-GNSS1A1 
expansion board with Teseo-LIV3F GNSS and X-NUCLEO-IKS01A3 expansion board with motion and
environmental sensors. The application relies on Amazon AWS Cloud IoT SDK, built over mbedTLS
and LwIP for secure communication.

The application connects to ST's Asset Tracker Dashboard frontend to present GNSS, Inertial 
and Environment Senors data to user. At the backend the application connects to Amazon 
AWS IoT Cloud. The credentials to connect to Dashboard and AWS IoT is downloaded from
ST's Asset Tracker Dashboard.

This application is very classic where the user is rested of all the complexities 
of IoT cloud configuration and quickly upto a production grade user interface to
face the data coming from its device.

A Firmware Other The Air (FOTA) update via HTTP(S) can be trigged by a specific AWS IoT job.

Upon a double-push of the User button, the application exits.

FOTA update notes:
- The application is launched by a BootLoader which also implements on-the-field firmware upgrade.
  For flashing the board locally in a single step, the IDE builds a complete image composed of
  the BootLoader executable and of the user application.
  This executable can be flashed and used for debugging the application (debug information are not present
  for the BootLoader part).
  The binary merge of the bootloader with the user application is done by post-build actions, after link stage.

  The BootLoader itself is generated from the X-CUBE-SBSFU STM32Cube expansion package,
  and provided as a pre-build executable.
  In order to enable application debug, the BootLoader is configured not to enforce the security features.
  Some specific directories of the X-CUBE-SBSFU package are included in the present package
  so that the BootLoader may be rebuilt with a different configuration;
  please refer to Projects/<board>/Applications/Cloud/AssetTracker/bootloader_readme.txt.

- SW4STM32 tools:
  When debugging with SW4STM32, the debugger starts the execution from the application main(), and
  not from the BootLoader main().
  A workaround is to perform a hard-reset on the board to force it to reboot correctly.


@par Directory contents

---> in .
Binary
  STM32L496G_AssetTracker.sfb        Pre-built secure image of the sample application. To be used in FOTA case.
  SBSFU_STM32L496G_AssetTracker.bin  Pre-built bootloader + sample application image.

Inc
  cmsis_os_misrac2012.h
  env_sensor.h                       Functions prototypes for the temperature driver.
  flash.h                            Management of the internal flash memory.
  FreeRTOSConfig.h                   FreeRTOS configuration.
  iks01a3_conf.h                     definitions for the MEMS components bus interfaces.
  ipc_config.h                       cellular IPC configuration
  main.h                             Configuration parameters for the application and globals.
  mbedtls_config.h                   Static configuration of the mbedTLS library.
  mbedtls_entropy.h                  mbedTLS RNG abstraction interface.
  motion_sensor.h                    functions prototypes for the accelerometer driver
  net_conf.h                         Configuration parameters for the STM32_Connect_Library.
  plf_config.h
  plf_features.h                     Cellular applications configuration (data mode, miscellaneous functionalities)
  plf_hw_config.h                    Cellular HW parameters (such as UART configuration, GPIO used, and others).
  plf_stack_size.h                   System thread stack sizes configuration (cellular and cloud thread)
                                     The stack sizes included in this file are used to calculate the FreeRTOS heap size
                                     (contained in file FreeRTOSConfig.h)
  plf_sw_config.h                    Cellular SW parameters configuration (task priorities, trace activations...)
  prj_config.h                       Cellular project file
  stm32l4xx_hal_conf.h               HAL configuration file.
  stm32l4xx_it.h                     STM32 interrupt handlers header file.
  stm32l496g_discovery_sensor.h      definitions for the MEMS Interface
  usart.h                            Usart declaration for cellular
  Patch\stm32l496g_discovery_patch.h definitions for STM32L496G_DISCOVERY's I2C1 Interface routines

Src
  board_interrupts.c                 Overriding of the native weak HAL interrupt base functions, for cellular usart
  flash_l4.c                         Flash programming interface.
  main.c                             Main application file.
  mbedtls_entropy.c                  mbedTLS RNG abstraction interface.
  net_conf.c                         Bus mapping of the STM32_Connect_Library to the eS_WiFi module.
  sensors_data.c                     Prepare data of X-Nucleo-IKS01A3 sensors.
  set_credentials.c                  Interactive provisionning of the network connection settings.
  stm32l4xx_hal_msp.c                Specific initializations.
  stm32l4xx_hal_timebase_tim.c       Overriding of the native weak HAL time base functions
  stm32l4xx_it.c                     STM32 interrupt handlers.
  stm32l496g_discovery_sensor.c      configuration of the sensor instances.
  system_stm32l4xx.c                 System initialization.
  usart.c                            Usart definition for cellular
  Patch\aws_subscribe_publish_sensor_values_patch.c Control of the measurement sampling and MQTT reporting loop
  Patch\stm32l496g_discovery_patch.c Set of firmware functions to interface I2C1.

---> in Projects/Misc_Utils
Inc
  cloud.h
  iot_flash_config.h
  msg.h                              Log interface
  rfu.h

Src
  cloud.c                            Cloud application init and deinit functions
  iot_flash_config.c                 Dialog and storage management utils for the user-configured settings.
  rfu.c                              Firmware update helpers.

---> in Middlewares/Third_Party/AWS
certs
  Amazon1_Comodo_Baltimore.crt       List of root CA certificates to be pasted on the board console at first launch.
                                       * AWS IoT device created _SINCE_ September 2018 use Amazon1;
                                       * the RTC init over HTTPS uses Comodo;
                                       * the remote firmware update file may be downloaded from Amazon S3,
                                         and then use Digicert Baltimore.

  Verisign_Comodo_Baltimore.crt      List of root CA certificates to be pasted on the board console at first launch.
                                       * AWS IoT device created _BEFORE_ September 2018 use Verisign;
                                       * the RTC init over HTTPS uses Comodo;
                                       * the remote firmware update file may be downloaded from Amazon S3,
                                         and then use Digicert Baltimore.

platform/STM32Cube
  aws_network_st_wrapper.c           AWS SDK network porting layer.
  aws_timer.c                        AWS SDK time porting layer.
  network_platform.h
  timer_platform.h

samples/STM32Cube
  aws_subscribe_publish_sensor_values.c  Sample application. Not used.
  aws_iot_config.h                   AWS IoT C SDK configuration.
  aws_version.h                      X-CUBE-AWS package version information.

---> in Utilities
CLD_utils/http_lib
  http_lib.c                         Light HTTP client
  http_lib.h

Time
  STM32CubeRTCInterface.c            Libc time porting to the RTC.
  timedate.c                         Initialization of the RTC from the network.
  timingSystem.c                     Libc time porting to the RTC.
  timedate.h
  timingSystem.h

---> in ../../BootLoader_OSC
2_Images_SBSFU/*                      Bootloader binary, and security settings (in SBSFU/App/app_sfu.h)
2_Images_SECoreBin/*                  Post-build scripts for the user application.
Linker_Common/*                       Image memory mapping definitions for the user application.


@par Hardware and Software environment
  - STM32L496G-Discovery board.

  - Quectel BG96 daughterboard connected to the MCU board IO expander: Modem package upside, SIM card slot downside.

    BG96 is the modem used by default. 

    See X-CUBE-CELLULAR package for further information: https://www.st.com/en/embedded-software/x-cube-cellular.html

  - (optional) Micro B USB cable, as additional power supply provided by the USB connected to cellular daughterboard.
    This may be interesting to have with computer having USB port not delivering enough power to STM32
    or sometimes in limit of cellular coverage. LED4 on STM32L496 blinks when current limitation occurs.

  - Activate the embedded SIM card physically integrated on the expansion cellular board
    * The kit comes with a 3-months free of charge subscription with 15MB of data using the embedded SIM.
    * The user must go to the stm32-c2c.com portal and enter a voucher to activate this subscription.
    * The Voucher is found by connecting the board to an hyperterminal as specified in the blister.
    * If the board needs to be reinitialized with the original firmware, this one is available
      at https://stm32-c2c.com, see "Restore factory firmware".
    * When the board has been recorded on the https://stm32-c2c.com portal with its voucher,
      proceed with the EMnify button available on this portal.

  - Active cellular SIM card fit in the slot.

    In order to activate the EMnify SIM card:
    * Follow the EMnify Setup Guide available in the Knowledge Base section of https://support.emnify.com

      Note for corporate users: Your card may have already been registered by the person who received the batch.
      You can skip the account creation, and:
      * get your email address added to the list of users of the EMnify corporate account;
      * click on the activation link you will receive by email;
      * login and activate your card from its IccID;
      * create your endpoint.
      See the details in "Enabling SIMs" section and below of the EMnify Setup Guide.

    If you are not sure if the IccID printed on your card is valid, you can cross-check with the "SIM Id (IccID):" value
    displayed on the console during the application startup.

  - A Cellular access point is required.

  - Modem Firmware upgrade
    * The Firmware modem (BG96) may need to be upgraded. Most of time it is preferable to migrate to the latest version.
    * Check and follow the x-cube-cellular Release Note (Cellular framework),
      available at https://www.st.com/en/embedded-software/x-cube-cellular.html (SW content)
    * Modem Firmware is within the Quectel xG96 SW upgrade zip file (with all instructions inside),
      available at https://stm32-c2c.com (in precompile demos repository)

  - BG96 bands selection
    * Network selection and radio access may take time at first usage.
    * It may be needed to optimize and accelerate the searching procedure.
    * It is done by tuning the BG96 modem Firmware with bands deployed in your region, country or by operator.
    * A BG96 cellular configuration tool is available at https://stm32-c2c.com
    
  - X-NUCLEO-IKS01A3 and X-NUCLEO-GNSS1A1
    Connect it to STM32L496G-Discovery board's Arduino™ Uno V3 connector at the back of the board.

  - ST's Asset Tracker Dashboard account. An account in this Dashboard must be created.
    See https://dsh-assettracking.st.com/

@par How to use it ?

For the application to work, you must do the following:

 - WARNING: Before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

 - WARNING: Before opening the project with any toolchain be sure signed up to
   ST's AWS Dashboard. This dashboard provides interface to register your IoT
   Device and download TLS Device certificate and key without which your device
   can't connect to dashboard. 
   
   - Open the IAR IDE and compile the project (see the release note for detailed
   information about the version). Alternatively you can use the Keil uVision
   toolchain (see the release note for detailed information about the version).
   Alternatively you can use the STM32CubeIDE for STM32 (see the release note
   for detailed information about the version).

 - Program the firmware on the STM32 board: If you generated a raw binary file,
   you can copy (or drag and drop) it from
   Projects\STM32L496G-Discovery\Applications\Cloud\AssetTracker\<toolchain>\PostBuild\SBSFU_<boardname>_AWS.bin
   to the USB mass storage location created when you plug the STM32 board to your PC.
   If the host is a Linux PC, the STM32 device can be found in the /media folder
   with the name e.g. "DIS_L496ZG". For example, if the created mass
   storage location is "/media/DIS_L496ZG", then the command to program the board
   with a binary file named "my_firmware.bin" is simply: cp my_firmware.bin
   /media/DIS_L4IOT.

   Alternatively, you can program the STM32 board directly through one of the
   supported development toolchains, the STM32 ST-Link Utility, or STM32CubeProgrammer.

   Note: If the board was programmed with an earlier version of the application,
   the Flash memory must be fully erased before re-programming, bBecause of possible change
   of the format of the user data stored in Flash.

 - Configure the required settings (to be done only once):
   - When the board is connected to a PC with USB (ST-LINK USB port),
     open a serial terminal emulator, find the board's COM port and configure it with:
      - 8N1, 115200 bauds, no HW flow control
      - set the line endings to LF or CR-LF (Transmit) and LF (receive).

   - At first boot, enter the requested parameters in the serial terminal emulator:

     - C2C network configuration (SIM operator access point code, username and password).
       Example:
         with Emnify SIM:  access point: "EM",     username: "", password: ""

     - Accept the default Amazon AWS IoT end-point (Amazon server address) as suggested in the terminal.
     
     - Accept the default device ID (First sixteen bytes of SIM id) as suggested in the terminal.
     
     - Accept the default root CA certificate as suggested in the terminal by Firmware.
     
     - Sign in to ST's Asset Tracker dashboard (http://st-asset-tracking.s3-website-us-east-1.amazonaws.com/#/signin),  enroll your device with suggested device id (First sixteen bytes of SIM id). 
     
     - Download your device TLS certificate from dashboard. The file contains TLS Device certificate and private key for your device.
     
     - Enter the  device certificate, and private key in the console when asked for.

       In order to use HTTPS for initializing the RTC, the root CA certificate of the target
       server must be provided together (in the same copy-paste) with the Amazon root CA certificate.
       You can use the provided file: Amazon1_Comodo_Baltimore.crt.

       If you use an old AWS IoT device created before September 2018, the root CA may be
       Symantec/Verisign instead of Amazon1. If so, Verisign_Comodo_Baltimore.crt must be used instead.

       HTTPS remote firmware update from AWS S3 is enabled by the third certificate contained in *_Comodo_Baltimore.crt.
       If you use your own HTTPS server instead of AWS S3, that third certificate must be replaced
       by the root CA of your web server.
       Note: File download over clear HTTP is also supported. Then no third certificate is needed.

  - By default the AWS sample application connects to ST's AWS dashboard as described above.
  - To visualize the data goto http://st-dashboard-iot-v2.s3-website-us-east-1.amazonaws.com/#/telemetry

   - After the parameters are configured, it is possible to change them by restarting the board
     and pushing User button (blue button) when prompted during the initialization.

@par Connectivity test
  - Makes an HTTPS request to retrieve the current time and date, and configures the RTC.

    Note: HTTPS has the advantage over NTP that the server can be authenticated by the board, preventing
          a possible man-in-the-middle attack.
          However, the first time the board is switched on (and each time it is powered down and up, if the RTC
          is not backed up), the verification of the server certificate will fail as its validity period will
          not match the RTC value.
          The following log will be printed to the console: "x509_verify_cert() returned -9984 (-0x2700)"

          If the CLOUD_TIMEDATE_TLS_VERIFICATION_IGNORE switch is defined in cloud.c (it is the case by default),
          this error is ignored and the RTC is updated from the "Date:" field of the HTTP response header.
          Else, a few more error log lines are printed, and the application retries to connect without verifying
          the server certificate.
          If/once ok, the RTC is updated.



@par Firmware upgrade

  The BootLoader allows to update the STM32 microcontroller built-in program with new firmware versions,
  adding new features and correcting potential issues. The update process can be performed in a secure way
  to prevent unauthorized updates and access to confidential on-device data. In addition, Secure Boot
  (Root of Trust services) checks and activates STM32 security mechanisms, and checks the authenticity and
  integrity of user application code before every execution to ensure that invalid or malicious code
  cannot be run. The Secure Firmware Update application receives the encrypted firmware image, checks its
  authenticity, decrypts it, and checks the integrity of the code before installing it.

  When building the AWS application a file containing the encrypted firmware image is generated:
  Projects/<board>/Applications/Cloud/AssetTracker/<toolchain>/PostBuild/<boardname>_AWS.sfb

  To test the firmware upgrade feature, place this file on an HTTP(S) server. Enter the URL of this file when
  prompted by the AWS application running on STM32 device (or use an AWS IoT job as described above).
  The encrypted firmware image is downloaded to device, the device reboots and checks
  its integrity before running the new firmware.

  It is possible to do a firmware update with an AWS IoT job. The syntax for the AWS IoT job document is:
    {
      "firmwareUpdate" : "http[s]://path.to/Firmware.sfb"
    }

  Here is an example using AWS S3 service for firmware storage:

  - you must have a device (board) flashed with an AWS firmware compiled with SBSFU build configuration
    and a .sfb SBSFU firmware file which version differs from the one of the device.

  - store the .sfb firmware file on AWS S3 service (in a S3 bucket)
    example: Project.sfb in bucket "bucket"
    make sure the file is readable by everyone (file properties / permissions) and is made public

  - create a text file "firmwareupdate.txt" with the following content and store it on S3:

    {
      "firmwareUpdate" : "https://s3.eu-central-1.amazonaws.com/bucket/Project.sfb"
    }

  - On AWS web portal, select the IoT Core service, go to Manage / Jobs
    * Click the "Create" button, then "create custom job"
    * Enter a job ID (number)
    * Select the device you wish to update
    * Select the job file you just uplodaded to S3: firmwareupdate.txt
    * Complete the job creation: Next / Create

  - Once the job is created and the device is connected to AWS IoT through MQTT, the device receives the job,
    the application exits the MQTT loop and starts downloading the new firmware file over HTTP(S).
    If successful, it reboots to let SBSFU update the firmware, and launch the new application.
    Once the new application runs and is connected to AWS IoT again, it verifies that its version
    has changed since the update job was received, and notifies AWS IoT of the job status: Completed, or Failed.


 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
