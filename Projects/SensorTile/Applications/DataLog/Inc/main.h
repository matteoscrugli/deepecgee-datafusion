/**
  ******************************************************************************
  * @file    main.h
  * @author  Central Labs
  * @version V2.0.0
  * @date    27-April-2017
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "SensorTile.h"
#include "SensorTile_accelero.h"
#include "SensorTile_gyro.h"
#include "SensorTile_magneto.h"
#include "SensorTile_pressure.h"
#include "SensorTile_temperature.h"
#include "SensorTile_humidity.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "console.h"
#include "osal.h"
#include "debug.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF

/* Exported macro ------------------------------------------------------------*/

/* Define the MOTENV1 MAC address, otherwise it will create a MAC related to STM32 UID */
//#define STATIC_BLE_MAC 0xFF, 0xEE, 0xDD, 0xAA, 0xAA, 0xAA

/*************** Debug Defines ******************/
/* Enable printf over USB Virtual COM Port */
/* Enabling this define for SensorTile will introduce a 10 Seconds delay */
#define DEBUG_USB_CONNECTION_INFO

/* For enabling connection and notification subscriptions debug */
#define DEBUG_USB_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define STLBLE_VERSION_MAJOR '1'
#define STLBLE_VERSION_MINOR '0'
#define STLBLE_VERSION_PATCH '0'

/* Define the Name, it MUST be 7 char long */
//#define NAME_STLBLE 'S','T','L','B',STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH
#define NAME_STLBLE 'O','L','O','K'
//#define NAME_STLBLE 'O','L','O','K',' ','J','e','s','t','e','r'

/* Package Name */
#define STLBLE_PACKAGENAME "STLBLE1"

    #include "usbd_cdc_interface.h"
//    #define STLBLE_PRINTF(...) {\
//      char TmpBufferToWrite[256];\
//      int32_t TmpBytesToWrite;\
//      TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
//      CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
//    }

char TmpBufferToWrite[256];
int32_t TmpBytesToWrite;
	#define STLBLE_PRINTF(...) {\
	  TmpBytesToWrite = sprintf( TmpBufferToWrite, __VA_ARGS__);\
	  CDC_Fill_Buffer(( uint8_t * )TmpBufferToWrite, TmpBytesToWrite);\
	}

/* STM32 Unique ID */

#define STM32_UUID ((uint32_t *)0x1FFF7590)


/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

/* Exported functions ------------------------------------------------------- */
extern void Error_Handler(void);

/* Exported defines and variables  ------------------------------------------------------- */

//10kHz/2 For Environmental data@2Hz
#define uhCCR1_Val  5000
//10kHz/20  For Acc/Gyromag@20Hz
#define uhCCR4_Val  500

  #define BLUEMSYS_CHECK_JUMP_TO_BOOTLOADER ((uint32_t)0x12345678)

extern uint8_t BufferToWrite[256];
extern int32_t BytesToWrite;

  
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
