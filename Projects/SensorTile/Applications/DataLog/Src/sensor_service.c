/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  Central LAB
  * @version V1.0.0
  * @date    21-Nov-2016
  * @brief   Add bluetooth services using vendor specific profiles.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "console.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "cmsis_os.h"

#include "SensorTile.h"

#define LED_ON 							BSP_LED_On(LED1)

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

TIM_HandleTypeDef    TimCCHandle;

extern uint8_t bdaddr[6];

extern uint32_t	taskState;
extern uint32_t SystemCoreClock;
extern uint32_t CycleTime;
extern uint32_t	samp_period_ms_index;
extern uint32_t	samp_period_ms_value;
extern uint32_t	send_period_ms_index;
extern uint32_t	send_period_ms_value;

extern osPoolId thread_poolId;
extern osMessageQId master_queueId;

osPoolId thread_poolId;
osMessageQId master_queueId;

/* Private variables ------------------------------------------------------------*/

typedef enum
{
	  TASK_SETUP = 0
	 ,SAMP_PERIOD_SETUP
	 ,SEND_PERIOD_SETUP
	 ,PERIOD_CHECK
} masterCommandTypeDef;


typedef enum
{
	 DATA_GENERIG	 = 20				// Generic data type
	,DATA_1D	= 7						// One-dimensional data type
	,DATA_3D	= 10					// Tri-dimensional data type

} sensorDataTypeTypeDef;

static uint16_t HWServW2STHandle;
uint16_t		allCharHandle[256];
int				CharHandleIndex = 0;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;
static uint16_t ConfigSclkCharHandle;
static uint16_t ConfigCycleCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

// static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;
  uint32_t NumberChars = 5;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*NumberChars,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_LED_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_CONFIG_SCLK_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 8,
		  CHAR_PROP_NOTIFY|CHAR_PROP_READ,
		                             ATTR_PERMISSION_NONE,
		                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
		                             16, 0, &ConfigSclkCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_CONFIG_CYCLE_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 8,
		  CHAR_PROP_NOTIFY|CHAR_PROP_READ,
		                             ATTR_PERMISSION_NONE,
		                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
		                             16, 0, &ConfigCycleCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //STLBLE_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    // HAL_Delay(10);
    osDelay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        STLBLE_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    // HAL_Delay(20);
    osDelay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}
/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  int32_t NumberChars = 9;

  uint8_t uuid[16];

  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  // uuid[11] += 6;
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*NumberChars,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /*
  // Fill the Environmental BLE Characteristc
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  uuid[11] += 6;
    if(TargetBoardFeatures.NumTempSensors==2) {
      uuid[14] |= 0x05; // Two Temperature values
      EnvironmentalCharSize+=2*2;
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      uuid[14] |= 0x04; // One Temperature value
      EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandleHumSensor) {
     uuid[14] |= 0x08; // Humidity
     EnvironmentalCharSize+=2;
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      uuid[14] |= 0x10; // Pressure value
      EnvironmentalCharSize+=4;
    }

    uuid[14] = 0xFF;
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize*2,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_LED_W2ST_CHAR_UUID(uuid);
  uuid[11] += 6;
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
 */
  return BLE_STATUS_SUCCESS;

fail:
  STLBLE_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
uint16_t addSensorCharacteristc(int sensorId, int sensorDataType)
{
  tBleStatus ret;

  uint8_t  EnvironmentalCharSize;
  uint16_t CharHandle;
  uint8_t uuid[16];
  uint8_t buf[2];


  STORE_LE_16(buf, sensorId);

//  EnvironmentalCharSize = sensorDataType;
  EnvironmentalCharSize = 20; // FIXME

  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  uuid[12] = buf[0];
  uuid[13] = buf[1];
  uuid[15] = sensorDataType;
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &CharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  allCharHandle[CharHandleIndex] = CharHandle;
  CharHandleIndex++;
  return CharHandle;


fail:
  STLBLE_PRINTF("Error while adding HW's Characteristcs service.\n\r");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 *
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  tBleStatus ret;
  uint8_t BuffPos;
  
  uint8_t buff[2+4+2+2+2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  
    if(TargetBoardFeatures.HandlePressSensor) {
      STORE_LE_32(buff + BuffPos,Press);
      BuffPos+=4;
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      STORE_LE_16(buff + BuffPos,Hum);
      BuffPos+=2;
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      STORE_LE_16(buff + BuffPos,Temp2);
      BuffPos+=2;

      STORE_LE_16(buff + BuffPos,Temp1);
      BuffPos+=2;
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      STORE_LE_16(buff + BuffPos,Temp1);
      BuffPos+=2;
    }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char, BLA\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Environmental Char, BLA\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}*/

/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus charUpdate(uint16_t CharHandle, int data_type, int sensor_type, uint32_t time, void *data, int data_len)
{
  tBleStatus ret;
  uint8_t BuffPos;
  uint8_t buff[sensor_type];
  uint16_t dataInt;
  uint8_t dataDec;

  STORE_LE_32(buff + BuffPos, time);
  BuffPos+=4;
  switch (data_type)
  {
	  case 2:
		  for(int i = 0; i < data_len; i++)
		  {
			  STORE_LE_16(buff + BuffPos, *(((int16_t*) data) + i));
			  BuffPos+=2;
		  }
		  for(int i = BuffPos; i < sensor_type; i++) buff[i] = 0;
		  break;

	  case 4:
		  for(int i = 0; i < data_len; i++)
		  {
			  STORE_LE_32(buff + BuffPos, *(((int32_t*) data) + i));
			  BuffPos+=4;
		  }
		  for(int i = BuffPos; i < sensor_type; i++) buff[i] = 0;
		  break;

	  case 5:
		  for(int i = 0; i < data_len; i++)
		  {
			  MCR_BLUEMS_F2I_2D(*(((float*) data) + i), dataInt, dataDec);

			  STORE_LE_16(buff + BuffPos, dataInt);
			  BuffPos+=2;

			  buff[BuffPos] = dataDec;
			  BuffPos+=1;
		  }
		  for(int i = BuffPos; i < sensor_type; i++) buff[i] = 0;
		  break;

	  case 12:
		  for(int i = 0; i < data_len; i++)
		  {
			  STORE_LE_16(buff + BuffPos, (int16_t) ((SensorAxes_t*)(data)+i)->AXIS_X);
			  BuffPos+=2;

			  STORE_LE_16(buff + BuffPos, (int16_t) ((SensorAxes_t*)(data)+i)->AXIS_Y);
			  BuffPos+=2;

			  STORE_LE_16(buff + BuffPos, (int16_t) ((SensorAxes_t*)(data)+i)->AXIS_Z);
			  BuffPos+=2;
		  }
		  for(int i = BuffPos; i < sensor_type; i++) buff[i] = 0;
		  break;
  }

//  STLBLE_PRINTF("\n");
//  for(int i = 0; i < sensor_type; i++) STLBLE_PRINTF("%x ", buff[i]);
//  STLBLE_PRINTF("\n");

  ret = aci_gatt_update_char_value(HWServW2STHandle, CharHandle, 0, sensor_type, buff);
  if (ret != BLE_STATUS_SUCCESS){
	BSP_LED_Toggle(LED1);
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
//  else BSP_LED_Toggle(LED1);
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 *
tBleStatus LED_Update(uint8_t LedStatus)
{
  tBleStatus ret;

  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = LedStatus;

  ret = aci_gatt_update_char_value(HWServW2STHandle, LedCharHandle, 0, 2+1,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating LED Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating Temp Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}*/

/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus sclkUpdate(void)
{
  tBleStatus ret;

  uint8_t buff[8];

  STORE_LE_32(buff  ,osKernelSysTick());
  STORE_LE_32(buff + 4, SystemCoreClock);

  ret = aci_gatt_update_char_value(ConfigServW2STHandle, ConfigSclkCharHandle, 0, 8, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating SystemCoreClock Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating SystemCoreClock Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update LEDs characteristic value
 * @param  uint8_t LedStatus LEDs status 0/1 (off/on)
 * @retval tBleStatus   Status
 */
tBleStatus cycleUpdate(uint32_t cycleTime)
{
  tBleStatus ret;

  uint8_t buff[4];

  STORE_LE_32(buff  ,cycleTime);

  ret = aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCycleCharHandle, 0, 4, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating SystemCoreClock Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      STLBLE_PRINTF("Error Updating SystemCoreClock Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_STLBLE};
  uint8_t manuf_data[26] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NAME_STLBLE, // Complete Name
    13,0xFF,0x01/*SKD version */,
    0x02,
    0x00, /* */
    0xE0, /* ACC+Gyro+Mag*/
    0x00, /*  */
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };

  /* BLE MAC */
  manuf_data[20] = bdaddr[5];
  manuf_data[21] = bdaddr[4];
  manuf_data[22] = bdaddr[3];
  manuf_data[23] = bdaddr[2];
  manuf_data[24] = bdaddr[1];
  manuf_data[25] = bdaddr[0];

  manuf_data[16] |= 0x20; /* Led */

  if(TargetBoardFeatures.HandleGGComponent){
    manuf_data[17] |= 0x02; /* Battery Present */
  }

    if(TargetBoardFeatures.NumTempSensors==2) {
      manuf_data[17] |= 0x05; /* Two Temperature values*/
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      manuf_data[17] |= 0x04; /* One Temperature value*/
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      manuf_data[17] |= 0x08; /* Humidity */
    }

    if(TargetBoardFeatures.HandlePressSensor) {
      manuf_data[17] |= 0x10; /* Pressure value*/
    }

    /* DS3 DIL24  present*/
    if(TargetBoardFeatures.HWAdvanceFeatures) {
      /* Accelerometer Events */
      manuf_data[18] |=0x04;
    }


  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
#ifndef STATIC_BLE_MAC
                           STATIC_RANDOM_ADDR,
#else /* STATIC_BLE_MAC */
                           PUBLIC_ADDR,
#endif /* STATIC_BLE_MAC */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(26, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef DEBUG_USB_CONNECTION_INFO
  STLBLE_PRINTF(">>>>>> CONNECTED >>>>>>\t MAC address: %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* ENABLE_USB_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
	int *message;
  connected = FALSE;

#ifdef DEBUG_USB_CONNECTION_INFO
  STLBLE_PRINTF("<<<<<< DISCONNECTED <<<<<<\t MAC address: \r\n");
#endif /* ENABLE_USB_DEBUG_CONNECTION */  

  /* Make the device connectable again. */
  set_connectable = TRUE;

  taskState = 0;
  message = osPoolAlloc(thread_poolId);
  *message = TASK_SETUP;
  osMessagePut(master_queueId, (uint32_t) message, osWaitForever);

  ConnectionBleStatus=0;

  /*
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    // Stopping Error
    Error_Handler();
  }*/
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  uint8_t Status;
  /*
  if(handle == EnvironmentalCharHandle + 1){
    // Read Request for Pressure,Humidity, and Temperatures
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;
      if(TargetBoardFeatures.HandlePressSensor) {
        if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
          BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
          MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
          PressToSend=intPart*100+decPart;
        }
      }

      if(TargetBoardFeatures.HandleHumSensor) {
        if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
          BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          HumToSend = intPart*10+decPart;
        }
      }

      if(TargetBoardFeatures.NumTempSensors==2) {
        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp1ToSend = intPart*10+decPart; 
        }

        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp2ToSend = intPart*10+decPart;
        }
      } else if(TargetBoardFeatures.NumTempSensors==1) {
        if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
          BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
          MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
          Temp1ToSend = intPart*10+decPart;
        }
      }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } else if(handle == LedCharHandle + 1){
    // Read Request for Led Status
    LED_Update(TargetBoardFeatures.LedStatus);
  }*/

  if(handle == ConfigSclkCharHandle + 1)
  {
	  sclkUpdate();
  }

  if(handle == ConfigCycleCharHandle + 1)
  {
	  cycleUpdate(CycleTime);							// TO MODIFY
  }

  else if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
  int *message;

  int charCheck = FALSE;
  if(attr_handle == ConfigCharHandle + 2) {
    ;/* do nothing... only for removing the message "Notification UNKNOW handle" */
  }
  /*
  else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      // Start the TIM Base generation in interrupt mode
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        // Starting Error
        Error_Handler();
      }

      // Set the new Capture compare value
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        // Set the Capture Compare Register value
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      // Stop the TIM Base generation in interrupt mode
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        // Stopping Error
        Error_Handler();
      }
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Env=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? "ON" : "OFF");
#endif // ENABLE_USB_DEBUG_CONNECTION
  }
  else if(attr_handle == LedCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_LED);
      // Update the LED feature
      LED_Update(TargetBoardFeatures.LedStatus);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_LED);
    }
#ifdef ENABLE_USB_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      STLBLE_PRINTF("--->Led=%s\r\n", W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED) ? "ON" : "OFF");
#endif // ENABLE_USB_DEBUG_CONNECTION
  }
  */

  if(attr_handle == ConfigSclkCharHandle + 2)
  {
		if (att_data[0] == 01) 	W2ST_ON_CONNECTION	(1 << 1);
		if (att_data[0] == 0) 	W2ST_OFF_CONNECTION	(1 << 1);
  }

  if(attr_handle == ConfigCycleCharHandle + 2)
  {
	  	if (att_data[0] == 01) 	W2ST_ON_CONNECTION	(1 << 2);
	  	if (att_data[0] == 0) 	W2ST_OFF_CONNECTION	(1 << 2);
  }

  else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  }
  else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);    
  } else {

	for(int i = 0; i < CharHandleIndex; i++)
	{
		if(attr_handle == allCharHandle[i] + 2)
		{
			if (att_data[0] == 01)
			{
				W2ST_ON_CONNECTION((1<<(12+i)));
				charCheck = TRUE;

				taskState |= (1 << i);
				STLBLE_PRINTF("TEST ON: %d, %d\r\n", i, taskState);

//				taskState |= (1 << ((i*2) + 1));
				message = osPoolAlloc(thread_poolId);
				*message = TASK_SETUP;
				osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
			}
			if (att_data[0] == 0)
			{
				W2ST_OFF_CONNECTION((1<<(12+i)));
				charCheck = TRUE;

				taskState &= ~(1 << i);
				STLBLE_PRINTF("TEST OFF: %d, %d\r\n", i, taskState);

//				taskState &= ~(1 << ((i*2) + 1));
				message = osPoolAlloc(thread_poolId);
				*message = TASK_SETUP;
				osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
			}
		}
	}
	if(!charCheck)
	{
	    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
	      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
	      Stderr_Update(BufferToWrite,BytesToWrite);
	    } else {
	      STLBLE_PRINTF("Notification UNKNOW handle\r\n");
	    }
	}

  }
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint8_t FeatureMask = (att_data[0]);
  uint8_t Command = att_data[1];
  uint32_t Data = 0;

  for(int i = 2; ((i < data_length) || (i < 6)); i++){
	  Data += ((att_data[i]) << (i - 2)*8);
  }

  int *message;

  switch (FeatureMask) {

   case FEATURE_MASK_TASK:
//     switch(Command) {
//
//       case 1:
//		 taskState |= (1 << (Data*2));
//			message = osPoolAlloc(thread_poolId);
//			*message = TASK_SETUP;
//			osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
//         break;
//
//       case 0:
//    	   taskState &= ~(1 << (Data*2));
//    	   message = osPoolAlloc(thread_poolId);
//    	   *message = TASK_SETUP;
//    	   osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
//
//    	   break;
//     }
	   taskState = Command;
	   message = osPoolAlloc(thread_poolId);
	   *message = TASK_SETUP;
	   osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
   break;



   case FEATURE_MASK_SAMP:
	   samp_period_ms_index = Command;
	   samp_period_ms_value = Data;
//	   for(int i = 0; i < (data_length - 2); i++) periodSamp += att_data[2+i]*(1 << 8*i);
	   message = osPoolAlloc(thread_poolId);
	   *message = SAMP_PERIOD_SETUP;
	   osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
//	   bpmControl = Command;
   break;



   case FEATURE_MASK_SEND:
	   send_period_ms_index = Command;
	   send_period_ms_value = Data;
//	   for(int i = 0; i < (data_length - 2); i++) periodSamp += att_data[2+i]*(1 << 8*i);
	   message = osPoolAlloc(thread_poolId);
	   *message = SEND_PERIOD_SETUP;
	   osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
//	   bpmControl = Command;
   break;

	default:
 	   ;
	break;

  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        if(TargetBoardFeatures.bnrg_expansion_board==IDB05A1) {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            } else {
              evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            }
        break;
      }
    }
    break;
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
