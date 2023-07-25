//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* NOTE

	...

*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	#define ARMVERSION						// sensor placed in the arm
//	#define DIRECTIONSVERSION 				// OLD OM



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INCLUDE

#include "main.h"

#include <stdio.h>
#include <math.h>
#include <limits.h>

#include "cmsis_os.h"
#include "arm_math.h"
#include "arm_nnfunctions.h"

#include "stm32l4xx_hal_pwr.h"

#include "datalog_application.h"		// REMOVE ME
#include "TargetFeatures.h"				// REMOVE ME

#ifdef ARMVERSION
#include "model_quantized_arm.h"
#endif

#ifndef ARMVERSION
#include "model_quantized_board.h"
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GENERAL DEFINE

	#define ENABLE_BLE
//	#define ENABLE_USB
//	#define ENABLE_LED


#define DATAQUEUE_SIZE  				((uint32_t) 50)

#define LED_ON 							BSP_LED_On(LED1)
#define LED_OFF 						BSP_LED_Off(LED1)
#define LED_TGG 						BSP_LED_Toggle(LED1)

#define timer_start()    				*((volatile uint32_t*)0xE0001000) = 0x40000001  						// Enable CYCCNT register
#define timer_stop()   					*((volatile uint32_t*)0xE0001000) = 0x40000000  						// Disable CYCCNT register
#define timer_get()   					*((volatile uint32_t*)0xE0001004)               						// Get value from CYCCNT register

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



typedef enum
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSORS

	 SENSOR_0 = 0

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// VARIOUS

	,THREAD_BLE							// BLE								Task name = 	bleThread
	,THREAD_MASTER						// Master Thread					Task name = 	MasterThread
	,MESSAGE_MASTER						// Edit FIFO
	,MESSAGE_SENSOR						// ...
	,MESSAGE_DELAYED					// ...
	,SEMAPHORE_BLE						// Timer semaphore
	,SEMAPHORE_MUTEX					// Semaphore mutex

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR_0

	,THREAD_GET_SENSOR_0				// Get sensor data					Task name = 	get_thread_0
	,THREAD_PROC1_SENSOR_0				// Balance state detection							proc1_thread_0
	,THREAD_PROC2_SENSOR_0				// Data classification								proc2_thread_0
	,THREAD_THRSH_SENSOR_0				//													thrsh_thread_1
	,THREAD_SEND_SENSOR_0				// Send sensor data									send_thread_0

	,MESSAGE_PROC1_SENSOR_0				// Proc2 FIFO
	,MESSAGE_PROC2_SENSOR_0				// Proc2 FIFO
	,MESSAGE_THRSH_SENSOR_0				// Threshold FIFO
	,MESSAGE_SEND_SENSOR_0				// Send FIFO

	,SEMAPHORE_SENSOR_0					// Get data semaphore

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



} systemTypeDef;

typedef enum
{
	  TASK_SETUP = 0
	 ,SAMP_PERIOD_SETUP
	 ,SEND_PERIOD_SETUP
	 ,PERIOD_CHECK
} masterCommandTypeDef;

typedef enum
{
	 READ_LAST_VALUE = 0
	,READ_FROM_FIFO
} messageTypeTypeDef;

typedef enum
{
	 DO_NOT_SEND = 0
	,SEND
} thresholdTypeTypeDef;

typedef enum							// To be removed
{
	 DATA_GENERIG	 = 20				// Generic data type
	,DATA_1D = 7						// One-dimensional data type
	,DATA_3D = 10						// Tri-dimensional data type

} sensorDataTypeTypeDef;

typedef enum
{
	 INT8	 = 1
	,INT16	 = 2
	,INT32	 = 4
	,FLOAT
	,AXISRAW = 6
	,AXIS	 = 12

} DataTypeTypeDef;

typedef enum
{
	 RUN = 0
	,LPRUN
	,SLEEP_WFI
	,SLEEP_WFE
	,LPSLEEP_WFI
	,LPSLEEP_WFE
	,STOP0_WFI
	,STOP0_WFE
	,STOP1_WFI
	,STOP1_WFE
	,STOP2_WFI
	,STOP2_WFE
	,STANDBY
	,SHUTDOWN
} powerModeTypeTypeDef;

//typedef struct
//{
//	int 				message_id;
//	int 				sensor_id;
//	uint16_t 			charHandle;
//
//	uint32_t 			ms_counter;
//
//	void 				*dataraw;
//	void 				*data;
//	int					data_len;
//} SensorsData;

typedef struct
{
	int 				message_id;
	int 				sensor_id;
	uint16_t 			charHandle;

	uint32_t 			ms_counter;

	void 				*dataraw;
	void 				*data;
	int					data_len;

//	osMessageQId 		out_get_queueId;
//	osMessageQId 		out_proc1_queueId;
//	osMessageQId 		out_proc2_queueId;
//	osMessageQId 		out_thrsh_queueId;
//	osMessageQId 		out_send_queueId;
//	bool 				consumerThread[] = {false, false, false, false, false};
} SensorsData;

struct sensorSetupStruct
{
	char 				name[20];
	char 				code[20];
	int				 	data_type;
	int 				data_len;
	int					sample_packing;
	int					sensor_type;
	uint16_t 			charHandle; // generic charHandle, DELETEME
	uint16_t 			charHandleRaw;
	uint16_t 			charHandleProcessing1;
	uint16_t 			charHandleProcessing2;

	osThreadId 			get_threadId;
	osThreadId 			proc1_threadId;
	osThreadId 			proc2_threadId;
	osThreadId 			thrsh_threadId;
	osThreadId 			send_threadId;

	osMessageQId 		proc1_queueId;
	osMessageQId 		proc2_queueId;
	osMessageQId 		thrsh_queueId;
	osMessageQId 		send_queueId;

	osMessageQId 		out_get_proc1_queueId;
	osMessageQId 		out_get_proc2_queueId;
	osMessageQId 		out_get_thrsh_queueId;
	osMessageQId 		out_get_send_queueId;

//	osMessageQId 		out_get_queueId;
	osMessageQId 		out_proc1_queueId;
	osMessageQId 		out_proc2_queueId;
	osMessageQId 		out_thrsh_queueId;
	osMessageQId 		out_send_queueId;

//	osMessageQId		outGetQueueId[5];
//	int 				en_threads[5];

	osSemaphoreId 		get_semId;

	osTimerId 			samp_timerId;
	int					samp_period_ms;
	void 				(*samp_timer_Callback) (void const *arg);

	osTimerId 			send_timerId;
	int					send_period_ms;
	void 				(*send_timer_Callback) (void const *arg);

	SensorsData 		*last_value;

	int			 		(*data_threshold) (SensorsData *data);

	DrvStatusTypeDef 	(*get_dataraw) (void *handle, SensorAxesRaw_t *data);
	DrvStatusTypeDef 	(*get_data) (void *handle, SensorAxes_t *data);
//	void 				(*init) (void);
//	void 				(*deInit) (void);
//	void 				(*enable) (void);
//	void 				(*disable) (void);
};

typedef struct
{
	unsigned int 		msDelay;
	osTimerId			timer_id;
	int 				sensor_id;
	osMessageQId 		receiver;
	SensorsData 		*message;
} DelayedSensorsDataMessage;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MASTER

#define 		INIT_MASTER_TIMER_PERIOD	1000

static void 	master_thread(				void const *argument);

osThreadId									master_threadId;
osThreadDef(								THREAD_MASTER, master_thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE * 2);

osMessageQId 								master_queueId;
osMessageQDef(								MESSAGE_MASTER, DATAQUEUE_SIZE, int);

osPoolId 									thread_poolId;
osPoolDef(									thread_pool, DATAQUEUE_SIZE, int);

void 			master_timer_Callback(		void const *arg);
void 			master_timer_Start(			void);
void 			master_timer_Stop(			void);

osTimerId 									master_timerId;
osTimerDef(									master_timer, master_timer_Callback);

uint32_t									timer_period_master;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BLE

#define INIT_BLE_TIMER_PERIOD				500

static void 	ble_thread(					void const *argument);
static void 	InitBlueNRGStack(			void);

osThreadId									ble_threadId;
osThreadDef(								THREAD_BLE, ble_thread, osPriorityNormal, 	0, 	configMINIMAL_STACK_SIZE * 4);

osSemaphoreId 								ble_semId;
osSemaphoreDef(								SEMAPHORE_BLE);

void 			ble_timer_Callback(			void const *arg);
void 			ble_timer_Start(			void);
void 			ble_timer_Stop(				void);

osTimerId 									ble_timerId;
osTimerDef(									ble_timer, ble_timer_Callback);

uint32_t									timer_period_ble;

void			delayedMessage_Callback(	void const *arg);



// CUSTOM PARAMETERS & FUNCTIONS:



#include "sensor_service.h"
#include "bluenrg_utils.h"

static void 	GetBatteryInfoData(			void);

uint32_t 									StartTime;
uint32_t 									EndTime;
uint32_t									CycleTime;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// VARIOUS

#define NUMBER_OF_SENSORS					1
#define INIT_TASK_STATE						0b00


void 			init_all_task(				void);
void 			*dataMalloc(				int type, int size);
void 			dataCopy(					int type, int *cnt, SensorsData *src, SensorsData *dst);

void 			Error_Handler(				void);

static void 	gp_read_from_sensor(		int sensorId);
static void 	gp_data_threshold(			int sensorId);
static void 	gp_send_to_gateway(			int sensorId);



osPoolId 									message_poolId;
osPoolDef(									MESSAGE_SENSOR, 		DATAQUEUE_SIZE, 	SensorsData);

osSemaphoreId 								mutex_semId;
osSemaphoreDef(								SEMAPHORE_MUTEX);

uint32_t									soc;
int32_t										current= 0;
uint32_t									voltage;

uint32_t 									timer_0;
uint32_t 									timer_1;
uint32_t 									timer_2;
uint32_t 									timer_3;
uint32_t 									timer_4;
uint32_t 									timer_5;
uint32_t 									timer_6;
uint32_t 									timer_7;
uint32_t 									timer_8;
uint32_t 									timer_9;

uint32_t 									timer_workload;
int 										workLoad = 0;
#define			MAX_WORKLOAD				255



extern uint32_t 							ulTimerCountsForOneTick;
extern uint8_t 								set_connectable;
extern int 									connected;
static volatile uint32_t 					HCI_ProcessEvent = 		0;

uint32_t									taskState = 			INIT_TASK_STATE;

uint8_t 									BufferToWrite[256];
int32_t 									BytesToWrite;
uint8_t 									bdaddr[6];
uint32_t 									ConnectionBleStatus = 	0;
uint32_t 									exec;
USBD_HandleTypeDef 							USBD_Device;

uint32_t									samp_period_ms_index;
uint32_t									samp_period_ms_value;

uint32_t									send_period_ms_index;
uint32_t									send_period_ms_value;

struct sensorSetupStruct 					sensorSetup[NUMBER_OF_SENSORS];



osMessageQId 								delayed_queueId;
osMessageQDef(								MESSAGE_DELAYED, DATAQUEUE_SIZE, DelayedSensorsDataMessage);



// CUSTOM PARAMETERS & FUNCTIONS:



void			powerControl(				int mode, int frequency);
static void 	GetBatteryInfoData(			void);
void			delayedOsMessagePut(		unsigned int msDelay, int sensor_id, osMessageQId receiver, SensorsData *message);


osTimerDef(delayedTimer, delayedMessage_Callback);
void *STC3115_handle = 						NULL;



#ifdef ARMVERSION
#define PROC1DOWNSAMPLE 2
#define PROC2DOWNSAMPLE 4
#define PROC2STEP		25
#endif

#ifndef ARMVERSION
#define PROC1DOWNSAMPLE 2
#define PROC2DOWNSAMPLE 7
#define PROC2STEP		14
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0

static void 	get_thread_0(				void const *argument);
static void 	proc1_thread_0(				void const *argument);
static void 	proc2_thread_0(				void const *argument);
static void 	thrsh_thread_0(				void const *argument);
static void 	send_thread_0(				void const *argument);

osMessageQDef(								MESSAGE_PROC1_SENSOR_0, DATAQUEUE_SIZE, 	int);
osMessageQDef(								MESSAGE_PROC2_SENSOR_0, DATAQUEUE_SIZE, 	int);
osMessageQDef(								MESSAGE_THRSH_SENSOR_0, DATAQUEUE_SIZE, 	int);
osMessageQDef(								MESSAGE_SEND_SENSOR_0, 	DATAQUEUE_SIZE, 	int);

osSemaphoreDef(								SEMAPHORE_SENSOR_0);

osThreadDef(								THREAD_GET_SENSOR_0,	get_thread_0,		osPriorityHigh,			0,	configMINIMAL_STACK_SIZE * 2); // 1
osThreadDef(								THREAD_PROC1_SENSOR_0,	proc1_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 2); // 1
osThreadDef(								THREAD_PROC2_SENSOR_0,	proc2_thread_0,		osPriorityNormal,		0,	configMINIMAL_STACK_SIZE * 4); // 2
osThreadDef(								THREAD_THRSH_SENSOR_0,	thrsh_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 1); // 1
osThreadDef(								THREAD_SEND_SENSOR_0,	send_thread_0,		osPriorityAboveNormal,	0,	configMINIMAL_STACK_SIZE * 4); // 4

void 			samp_timer_0_Callback(		void const *arg);
void 			send_timer_0_Callback(		void const *arg);



// CUSTOM PARAMETERS & FUNCTIONS:


void 			*LSM303AGR_X_0_handle = 	NULL;
DrvStatusTypeDef BSP_ACCELERO_GetData( 		void *handle, SensorAxes_t *acceleration );
void 			enableAcc( 					void );
static void 	initializeAcc( 				void );

void 			*LSM6DSM_G_0_handle = 		NULL;
void 			enableGyro( 				void );
static void 	initializeGyro( 			void );

#define 		AVERAGE_SAMPLE 				4 // must be a power of 2, FIXME
#define 		DOWNSAMPLE 					5
#define			DIRECTIONS					4

int16_t 		procDataX[					AVERAGE_SAMPLE];
int16_t 		procDataY[					AVERAGE_SAMPLE];

#define FMODE1 // AVERAGE
//#define FMODE2 // MEDIAN

//#define M2

#define HORIZONTAL
//#define VERTICAL

//int16_t 		minRotX[					AVERAGE_SAMPLE];
//int16_t 		maxRotX[					AVERAGE_SAMPLE];
//int16_t 		minRotY[					AVERAGE_SAMPLE];
//int16_t 		maxRotY[					AVERAGE_SAMPLE];

void swap(int16_t* xp, int16_t* yp)
{
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Function to perform Selection Sort
void selectionSort(int16_t arr[], int16_t n)
{
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {

        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;

        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
    }
}

int			 	data_threshold(				SensorsData *data);

static void 	balance_state_detector(		int sensorId);
static void 	data_classifier(			int sensorId);

#define 		RAW_SAMP_PERIOD_VALUE		10 // * 5 // 10
#define 		PROC_SEND_PERIOD_VALUE		1000
#define 		CNN_DOWNSCALING				1 // * 5// 5



#define 	CNN_20_100



#ifdef		CNN_20_100



#ifndef		PYTORCH_MODEL
#define		QUANT_SCALE			4.5
#define		CONV1_IN_DIM 		215
#define		CONV1_IF 			2
#define		CONV1_OF 			20
#define		CONV1_K_DIM 		9
#define 	CONV1_SCALE			4.5
#define 	CONV1_WEIGHT_SCALE	4.5

#define 	POOL_K_DIM 			2

#define		CONV2_IF 			20
#define		CONV2_OF 			20
#define		CONV2_K_DIM 		9
#define 	CONV2_SCALE			4.5
#define 	CONV2_WEIGHT_SCALE	4.5

#define 	FC1_IN_DIM			940
#define 	FC1_OUT_DIM			100
#define 	FC1_SCALE			4.5
#define 	FC1_PACKED_PARAMS_WEIGHT_SCALE	4.5

//#define 	RELU3_IN_DIM		// VALUE

#define 	FC2_IN_DIM			100
#define 	FC2_OUT_DIM			5
#define 	FC2_SCALE			4.5
#define 	FC2_PACKED_PARAMS_WEIGHT_SCALE	4.5
#endif



#define		CONV1_OUT_DIM 		(CONV1_IN_DIM + 1 - CONV1_K_DIM) // * CONV1_OF
#define		CONV1_STRIDE 		1
#define		CONV1_PADDING 		0
#define		CONV1_BIAS_LSHIFT 	0
#define		CONV1_RSHIFT		round(log(CONV1_SCALE) / log(2))

#define 	RELU1_IN_DIM		(CONV1_OUT_DIM * CONV1_OF)

#define 	POOL1_IN_DIM		CONV1_OUT_DIM // * CONV1_OF
#define 	POOL1_IF			CONV1_OF
#define 	POOL1_K_DIM			POOL_K_DIM
#define 	POOL1_STRIDE		2
#define 	POOL1_PADDING		0
#define 	POOL1_OUT_DIM		(unsigned int) (CONV1_OUT_DIM / POOL1_K_DIM) //  * CONV1_OF


#define 	CONV2_IN_DIM		(unsigned int) (POOL1_OUT_DIM)
#define		CONV2_OUT_DIM 		((POOL1_OUT_DIM) + 1 - CONV2_K_DIM) // * CONV2_OF
#define		CONV2_STRIDE 		1
#define		CONV2_PADDING 		0
#define		CONV2_BIAS_LSHIFT 	0
#define		CONV2_RSHIFT		round(log(CONV2_SCALE) / log(2))

#define 	RELU2_IN_DIM		(CONV2_OUT_DIM * CONV1_OF)

#define 	POOL2_IN_DIM		CONV2_OUT_DIM // * CONV1_OF
#define 	POOL2_IF			CONV2_OF
#define 	POOL2_K_DIM			POOL_K_DIM
#define 	POOL2_STRIDE		2
#define 	POOL2_PADDING		0
#define 	POOL2_OUT_DIM		(unsigned int) (CONV2_OUT_DIM) //  * CONV1_OF

#define 	FC1_BIAS_LSHIFT		0
#define 	FC1_OUT_RSHIFT		round(log(FC1_SCALE) / log(2))

#define 	RELU3_IN_DIM		FC1_OUT_DIM

#define 	FC2_BIAS_LSHIFT		0
#define 	FC2_OUT_RSHIFT		round(log(FC2_SCALE) / log(2))

#define		COLBUFFER_DIM		2 * FC1_IN_DIM// 2 * CONV1_IF * CONV1_K_DIM	// maximum between:    FULLY-> FCN_IN_DIM    CONV-> 2 * CONVN_IF * CONVN_K_DIM
#define		CNNBUFFER			CONV1_OUT_DIM * CONV1_OF	// maximum between:    FULLY-> FCN_IN_DIM    CONV-> CONVN_IF * CONVN_IN_DIM       CONV-> CONVN_OF * CONVN_OUT_DIM

//	#define		DUMMYINPUT
//	#define		DUMMYWTBIAS

//	#define	OUTPUT_SHIFT

	#define	CONV1_BASIC
	#define	CONV2_BASIC
	#define	FULLY1_BASIC
	#define	FULLY2_BASIC

	#define	CONV1_RAM
	#define	CONV2_RAM
//	#define	FC1_RAM
	#define	FC2_RAM



#ifndef DUMMYINPUT
	q7_t cnn_input [CONV1_IN_DIM * CONV1_IF * 2] = {0};
	q7_t *input; //[CONV1_IN_DIM * CONV1_IF];
#else
//	q7_t *input;
//	/* 0 test */	q7_t input[] = {-37, 17, 22, -11, 27, 19, -4, 35, 4, -2, 41, -11, -7, 39, -4, -13, 51, 15, -20, 59, 20, -18, 64, 4, -10, 69, 3, -15, 65, 14, -34, 47, 21, -32, 25, 24, -16, 12, 11, -18, 4, 11, -20, 7, 9, -27, -5, 27, -37, -10, 30, -49, -14, 40, -64, -19, 26, -75, -36, 30, -90, -43, 37, -107, -49, 38, -115, -50, 38, -102, -32, 34, -81, -22, 32, -63, -13, 40, -54, -6, 34, -62, 8, 30, -45, 26, 36, -26, 25, 40, -39, 23, 30, -25, 25, 13, -9, 32, -2, -7, 38, -15, -5, 49, -6, -21, 65, 15, -25, 66, 15, -18, 60, 6, -18, 64, 10, -25, 63, 25, -36, 43, 29, -29, 21, 24, -15, -2, 1, -15, -7, -8, -23, -8, 12, -35, -4, 36, -36, -9, 29, -59, -7, 44, -58, -12, 34, -62, -20, 32, -71, -28, 32, -89, -41, 37, -117, -54, 44, -111, -48, 34, -81, -33, 43, -60, -17, 40, -63, -11, 39, -47, -6, 37, -50, 5, 38, -41, 21, 38, -42, 36, 29, -30, 27, 30, -21, 23, 14, -10, 32, 8, -6, 42, -11, -5, 42, -12, -14, 50, 9, -26, 63, 25, -19, 62, 9, -20, 61, 3, -9, 64, 14, -22, 58, 22, -31, 39, 24, -17, 10, 17, -12, 1, 3, -20, 12, 4};
//	/* 1 test */	q7_t input[] = {-23, 53, 10, -26, 53, 8, -22, 68, -4, -20, 65, 1, -22, 57, 2, -20, 56, 4, -21, 59, 1, -17, 59, 0, -18, 61, -1, -17, 59, -1, -17, 59, -1, -18, 60, -1, -17, 60, 0, -18, 58, 1, -19, 59, 0, -19, 58, 0, -22, 59, 2, -33, 55, 1, -39, 56, 1, -38, 57, 6, -40, 57, 10, -34, 57, 20, -29, 44, 23, -22, 29, 26, -25, 17, 23, -36, 12, 15, -33, 2, 16, -35, -5, 17, -41, -4, 21, -46, -8, 25, -49, -8, 28, -57, -9, 24, -54, -17, 19, -66, -21, 28, -66, -34, 30, -69, -42, 27, -70, -51, 31, -96, -52, 43, -94, -53, 39, -83, -50, 32, -76, -42, 31, -64, -30, 29, -54, -29, 23, -48, -19, 23, -38, -19, 17, -35, -14, 17, -37, -8, 16, -47, 1, 17, -44, 1, 19, -49, 7, 20, -52, 16, 23, -48, 21, 23, -49, 31, 24, -47, 39, 23, -42, 44, 20, -38, 46, 20, -38, 44, 20, -35, 46, 19, -32, 48, 18, -31, 52, 15, -27, 53, 13, -26, 52, 10, -24, 57, 9, -28, 52, 14, -25, 57, 12, -22, 64, 1, -19, 60, 4, -19, 60, 2, -21, 57, 6, -20, 60, 1, -18, 57, 2, -20, 57, 2, -18, 59, 2, -20, 60, 2, -20, 60, 3, -19, 57, 3, -23, 53, 10, -26, 53, 8, -22, 68, -4, -20, 65, 1, -22, 57, 2, -20, 56, 4, -21, 59, 1, -17, 59, 0, -18, 61, -1, -17, 59, -1, -17, 59, -1, -18, 60, -1, -17, 60, 0, -18, 58, 1, -19, 59, 0, -19, 58, 0, -22, 59, 2, -33, 55, 1, -39, 56, 1, -38, 57, 6, -40, 57, 10, -34, 57, 20, -29, 44, 23, -22, 29, 26, -25, 17, 23, -36, 12, 15, -33, 2, 16, -35, -5, 17, -41, -4, 21, -46, -8, 25, -49, -8, 28, -57, -9, 24, -54, -17, 19, -66, -21, 28, -66, -34, 30, -69, -42, 27, -70, -51, 31, -96, -52, 43, -94, -53, 39, -83, -50, 32, -76, -42, 31, -64, -30, 29, -54, -29, 23, -48, -19, 23, -38, -19, 17, -35, -14, 17, -37, -8, 16, -47, 1, 17, -44, 1, 19, -49, 7, 20, -52, 16, 23, -48, 21, 23, -49, 31, 24, -47, 39, 23, -42, 44, 20, -38, 46, 20, -38, 44, 20, -35, 46, 19, -32, 48, 18, -31, 52, 15, -27, 53, 13, -26, 52, 10, -24, 57, 9, -28, 52, 14, -25, 57, 12, -22, 64, 1, -19, 60, 4, -19, 60, 2, -21, 57, 6, -20, 60, 1, -18, 57, 2, -20, 57, 2, -18, 59, 2, -20, 60, 2, -20, 60, 3, -19, 57, 3};
//	/* 1 ok_2 */	q7_t
//	/* 1 ok_3 */	q7_t input[] = {-18, 59, 14, -18, 58, 14, -18, 57, 14, -18, 57, 14, -18, 58, 14, -18, 57, 14, -19, 57, 14, -18, 58, 14, -18, 58, 14, -19, 57, 14, -18, 56, 14, -19, 57, 14, -19, 57, 14, -20, 57, 14, -19, 57, 14, -19, 57, 14, -18, 57, 14, -19, 57, 15, -24, 59, 14, -25, 61, 15, -21, 62, 14, -24, 59, 21, -30, 52, 24, -30, 48, 25, -28, 42, 21, -24, 29, 18, -21, 23, 17, -23, 19, 21, -28, 17, 26, -32, -2, 21, -43, -1, 37, -46, 1, 45, -56, -7, 39, -53, -12, 35, -63, -12, 41, -63, -24, 41, -70, -28, 52, -77, -34, 60, -84, -29, 62, -79, -32, 56, -72, -22, 54, -65, -12, 52, -60, -8, 48, -53, -9, 42, -43, -7, 40, -40, 12, 32, -45, 16, 39, -40, 18, 34, -32, 21, 29, -22, 23, 17, -16, 26, 15, -22, 26, 21, -19, 39, 18, -20, 54, 14, -16, 69, 7, -16, 70, 9, -19, 63, 10, -19, 55, 14, -16, 54, 14, -16, 56, 14, -17, 60, 11, -16, 61, 8, -15, 59, 8, -16, 58, 10, -17, 56, 11, -17, 54, 12, -20, 55, 13, -20, 56, 12, -20, 57, 12, -20, 59, 11, -19, 60, 11, -18, 60, 9, -18, 60, 11, -18, 59, 11, -18, 59, 12, -17, 59, 11};
//	/* 0 ok_4 */	q7_t input[] = {-21, 54, -9, -11, 59, -22, -18, 62, -9, -8, 55, -4, -12, 59, -18, -24, 64, -10, -14, 57, 0, -6, 59, -11, -16, 61, 1, -10, 56, -4, -9, 58, -1, -15, 56, -9, -13, 63, 3, -13, 63, 14, -9, 62, 0, -17, 61, 9, -13, 57, 9, -10, 59, 11, -15, 61, 10, -4, 61, 6, -11, 62, 4, -7, 61, 5, -7, 61, 6, -9, 62, 1, -9, 62, 3, 1, 60, 4, -3, 62, 3, 1, 59, -5, -11, 61, -1, -13, 63, 2, -4, 63, -1, -5, 63, -12, -12, 65, -17, -16, 71, -15, -6, 57, -17, -12, 49, -17, -9, 36, -17, -1, 18, -28, 80, 127, -28, -20, -45, 47, 46, 92, -15, 2, 54, -5, 7, 61, -7, -11, 65, 1, 3, 62, -4, 8, 64, 9, -2, 60, 5, -3, 49, 7, 0, 61, 6, 4, 59, 11, 1, 62, 7, -5, 62, 11, -4, 60, -1, -15, 61, 0, -16, 61, -3, -14, 56, -4, -15, 58, -2, -38, 59, -6, -17, 58, -12, -41, 55, -26, -9, 52, -8, 0, 61, -32, -13, 50, -60, -14, 58, -19, -23, 58, -17, -20, 52, -21, -8, 56, -19, -22, 54, -23, -18, 59, -15, -20, 50, -4, -19, 56, -19, -15, 61, -13, -33, 66, -11, -11, 55, -5, -20, 56, -9, -14, 58, -8};
//	/*  pt */		q7_t input[] = {-29, 52, 17, -29, 52, 18, -29, 52, 18, -29, 52, 17, -29, 51, 18, -30, 51, 19, -37, 47, 19, -35, 50, 16, -59, 49, 38, -45, 39, 30, -39, 32, 39, -34, 22, 37, -15, 18, 43, -27, 6, 30, -33, 1, 12, -49, -16, 11, -33, -44, 27, -47, -25, 40, -40, -38, 38, -63, -91, 57, -76, -101, 58, -82, -89, 55, -60, -76, 50, -58, -56, 42, -38, -19, 28, -29, -9, 27, -20, -6, 23, -31, -6, 27, -29, 8, 19, -43, 21, 27, -54, 31, 33, -44, 32, 34, -41, 35, 24, -32, 55, 19, -30, 65, 5, -30, 54, 14, -32, 51, 20, -28, 50, 18, -28, 52, 18, -29, 54, 19, -26, 53, 17, -26, 53, 18, -28, 53, 18, -27, 52, 18, -28, 52, 17, -28, 52, 18, -29, 53, 18, -30, 53, 18, -28, 51, 17, -30, 51, 18, -30, 51, 18, -29, 52, 17, -30, 52, 18, -30, 52, 18, -30, 52, 17, -29, 52, 17, -29, 52, 18, -29, 52, 18, -29, 52, 18, -29, 52, 17, -30, 52, 18};
//	/* fix */		q7_t input[] = {-98, -41, 19, -113, -41, 20, -109, -38, 14, -96, -23, 25, -74, -5, 33, -73, -4, 24, -64, 14, 20, -52, 27, 26, -61, 32, 26, -34, 37, 17, -25, 32, 13, -14, 36, 7, -6, 41, -10, -1, 37, -19, -5, 37, -11, -13, 43, 4, -23, 57, 13, -24, 61, 4, -11, 57, -9, -9, 66, -13, -10, 70, -3, -23, 60, 12, -25, 45, 18, -21, 17, 6, -25, 5, 6, -31, 7, 18, -39, 8, 26, -46, 9, 23, -51, 5, 20, -53, 2, 22, -61, -5, 23, -66, -11, 24, -68, -23, 18, -74, -32, 18, -85, -42, 19, -113, -51, 23, -112, -42, 21, -78, -14, 28, -74, -1, 38, -74, 2, 36, -68, 19, 35, -52, 29, 24, -33, 33, 17, -26, 34, 6, -9, 35, 0, -3, 38, -9, 0, 42, -11, -6, 42, -7, -18, 53, 13, -17, 65, -2, -9, 59, -18, -10, 58, -16, -19, 69, -9, -8, 67, 7, -29, 56, 10, -27, 48, 14, -25, 32, 13, -25, 5, 10, -30, 3, 9, -38, 2, 10, -45, 1, 22};

	/* bb */		q7_t input[] = {3, 2, 2, 2, 4, 2, 5, 3, -1, -1, -3, -4, -3, 4, 0, 4, -2, 1, -2, -1, 2, 1, 4, 0, 5, -3, 0, -3, -3, -4, -4, -3, -1, -5, 0, -9, 0, -7, -1, -10, -1, -11, 0, -8, 5, -12, 1, -6, 2, -9, 1, -10, 2, -8, -4, -13, -6, -18, -6, -16, -3, -10, 0, -15, -1, -15, 1, -15, 1, -16, -4, -18, -2, -16, -5, -17, -3, -19, -6, -17, -5, -18, -2, -17, -2, -20, 0, -21, -1, -21, 0, -23, -1, -13, -2, -26, -1, -27, -1, -27, -1, -27, -1, -27, 0, -27, -3, -26, -2, -27, -7, -26, -6, -26, -8, -25, -6, -26, -5, -26, -5, -26, -9, -26, -6, -24, -11, -25, -2, 11, -1, -1, -4, -8, -3, -5, -4, -13, 0, -6, 2, -6, 1, -8, -5, 0, -7, 4, -5, 0, -1, -1, -1, 3, -1, 2, -3, 2, -3, 3, -3, 2, 0, 4, -1, 6, -2, 9, -3, 8, -6, 11, -6, 13, -6, 18, -6, 15, -5, 20, -9, 19, -9, 19, -6, 21, -6, 19, -6, 22, -11, 21, -9, 21, -10, 22, -4, 26, -2, 23, 0, 24, 0, 24, 0, 24, 6, 24, 3, 24, 6, 24, 7, 24, 7, 23, 9, 22, 8, 22, 9, 22, 9, 23, 6, 23, 6, 23, 5, 23, 7, 23, 6, 23, 4, 24, 6, 24, 3, 24, 0, 24, 3, 24, 2, 24, 4, 23, 5, 24, 4, 24, 6, 24, 5, 23, 6, 23, 4, 24, 4, 24, 4, 24, 6, 23, 6, 24, 5, 25, 3, 22, 4, 27, 4, 19, 4, 16, 4, 12, 4, 9, 4, 9, 3, 13, 2, 14, 3, 11, 2, 14, 4, 10, 4, 8, 4, 9, 3, 9, 1, 10, 4, 7, 1, 7, 1, 3, -3, 2, -1, 2, 1, 7, 4, 4, 1, -1, 3, -3, 1, -10, 1, -12, 4, -11, -2, -9, -1, -11, -2, -7, 0, -5, 5, -9, 0, -6, 0, -11, -4, -15, -7, -12, -8, -12, -3, -12, -5, -2, -3, -3, -6, -4, -7, -6, -6, -7, -3, -8, 0, -14, 0, -19, 3, -20, -1, -24, -4, -26, -1, -19, -6, -23, -3, -23, -6, -21, -4, -25, -8, -24, -5, -26, -6, -26, -8, -26, -8, -25, -8, -25, -7, -26, -9, -24, -8, -25, -10, -25, -8, -25, -5, -27, -6, -26, -8, -25, -10, -24, -10, -24, -9, -25, -5, -26, 0, -27, -6, -27, -3, -26, -7, -26, -4, -30, -3, -21, 0, -24};
#endif

	q7_t 		*conv1_input;
	q7_t 		*conv1_output;

#if not defined CONV1_RAM && not defined DUMMYWTBIAS
	const q7_t 	conv1_wt[] = 		CONV1_WEIGHT;
	const q7_t 	conv1_bias[] = 		CONV1_BIAS;
#endif

#if defined CONV1_RAM && not defined DUMMYWTBIAS
	q7_t 		conv1_wt[] = 		CONV1_WEIGHT;
	q7_t 		conv1_bias[] = 		CONV1_BIAS;
#endif

#if not defined CONV1_RAM && defined DUMMYWTBIAS
	const q7_t 	conv1_wt[CONV1_IF * CONV1_OF * CONV1_K_DIM] = {0};
	const q7_t 	conv1_bias[CONV1_OF] = {0};
#endif

#if defined CONV1_RAM && defined DUMMYWTBIAS
	q7_t 		conv1_wt[CONV1_IF * CONV1_OF * CONV1_K_DIM];
	q7_t 		conv1_bias[CONV1_OF];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*conv1_output_o32;
#endif



	q7_t		*relu1_inout;



	q7_t		*maxpool1_inout;



	q7_t 		*conv2_input;
	q7_t 		*conv2_output;

#if not defined CONV2_RAM && not defined DUMMYWTBIAS
	const q7_t 	conv2_wt[] = 		CONV2_WEIGHT;
	const q7_t 	conv2_bias[] = 		CONV2_BIAS;
#endif

#if defined CONV2_RAM && not defined DUMMYWTBIAS
	q7_t 		conv2_wt[] = 		CONV2_WEIGHT;
	q7_t 		conv2_bias[] = 		CONV2_BIAS;
#endif

#if not defined CONV2_RAM && defined DUMMYWTBIAS
	const q7_t 	conv2_wt[CONV2_IF * CONV2_OF * CONV2_K_DIM] = {0};
	const q7_t 	conv2_bias[CONV2_OF] = {0};
#endif

#if defined CONV2_RAM && defined DUMMYWTBIAS
	q7_t 		conv2_wt[CONV2_IF * CONV2_OF * CONV2_K_DIM];
	q7_t 		conv2_bias[CONV2_OF];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*conv2_output_o32;
#endif



	q7_t		*relu2_inout;



	q7_t		*maxpool2_inout;



	q7_t		*fc1_input;
	q7_t		*fc1_output;

#if not defined FC1_RAM && not defined DUMMYWTBIAS
	const q7_t 	fc1_wt[] = 			FC1_PACKED_PARAMS_WEIGHT;
	const q7_t 	fc1_bias[] = 		FC1_PACKED_PARAMS_BIAS;
#endif

#if defined FC1_RAM && not defined DUMMYWTBIAS
	q7_t 		fc1_wt[] = 			FC1_PACKED_PARAMS_WEIGHT;
	q7_t 		fc1_bias[] = 		FC1_PACKED_PARAMS_BIAS;
#endif

#if not defined FC1_RAM && defined DUMMYWTBIAS
	const q7_t 	fc1_wt[FC1_IN_DIM * FC1_OUT_DIM] = {0};
	const q7_t 	fc1_bias[FC1_OUT_DIM] = {0};
#endif

#if defined FC1_RAM && defined DUMMYWTBIAS
	q7_t 	fc1_wt[FC1_IN_DIM * FC1_OUT_DIM];
	q7_t 	fc1_bias[FC1_OUT_DIM];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*fc1_output_o32;
#endif



	q7_t		*relu3_inout;



	q7_t		*fc2_input;
	q7_t		*fc2_output;

#if not defined FC2_RAM && not defined DUMMYWTBIAS
	const q7_t 	fc2_wt[] = 			FC2_PACKED_PARAMS_WEIGHT;
	const q7_t 	fc2_bias[] = 		FC2_PACKED_PARAMS_BIAS;
#endif

#if defined FC2_RAM && not defined DUMMYWTBIAS
	q7_t 		fc2_wt[] = 			FC2_PACKED_PARAMS_WEIGHT;
	q7_t 		fc2_bias[] = 		FC2_PACKED_PARAMS_BIAS;
#endif

#if not defined FC2_RAM && defined DUMMYWTBIAS
	const q7_t 	fc2_wt[FC2_IN_DIM * FC2_OUT_DIM] = {0};
	const q7_t 	fc2_bias[FC2_OUT_DIM] = {0};
#endif

#if defined FC2_RAM && defined DUMMYWTBIAS
	q7_t 	fc2_wt[FC2_IN_DIM * FC2_OUT_DIM];
	q7_t 	fc2_bias[FC2_OUT_DIM];
#endif

#ifdef OUTPUT_32BIT
	q31_t 		*fc2_output_o32;
#endif



	q15_t 			col_buffer[COLBUFFER_DIM] = {0};

	q7_t 			cnn_a_buffer[CNNBUFFER] = {0};
	q7_t 			cnn_b_buffer[CNNBUFFER] = {0};



	int 			fc2_output_maxindex;
	q7_t 			fc2_output_max;
	char 			classes[] = {'A', 'B', 'C', 'D', 'E'};

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MAIN

	HAL_Init();

	SystemClock_Config();
//	SystemClock_Config_lp();
//	powerControl(RUN,4);
//	powerControl(LPRUN, 2);

//	BSP_LED_Init(LED1);
//	LED_ON;

	HAL_PWREx_EnableVddUSB();													// Enable USB power on Pwrctrl CR2 register
	USBD_Init(								&USBD_Device, &VCP_Desc, 0);		// Init Device Library
	USBD_RegisterClass(						&USBD_Device, USBD_CDC_CLASS);		// Add Supported Class
	USBD_CDC_RegisterInterface(				&USBD_Device, &USBD_CDC_fops);		// Add Interface Callbacks for AUDIO and CDC Class
	USBD_Start(								&USBD_Device);						// Start Device Process

// 	InitTargetPlatform(TARGET_SENSORTILE);
	InitBlueNRGStack();

	initializeGyro();
//	enableGyro();
	initializeAcc();
	enableAcc();

//	static DrvContextTypeDef ACCELERO_SensorHandle[ ACCELERO_SENSORS_MAX_NUM ];
//	static ACCELERO_Data_t ACCELERO_Data[ ACCELERO_SENSORS_MAX_NUM ]; // Accelerometer - all.
//	static LSM303AGR_X_Data_t LSM303AGR_X_0_Data; // Accelerometer - sensor 1.
//
//	uint8_t data = 0x01; // +-2g 0x1, +-4g 0x11, +-8g 0x21, +-16g 0x31
//
//	/* Setup sensor handle. */
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].who_am_i      = LSM303AGR_ACC_WHO_AM_I;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].ifType        = 1; // SPI interface
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].address       = LSM303AGR_ACC_I2C_ADDRESS;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].spiDevice     = LSM303AGR_X;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].instance      = LSM303AGR_X_0;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isInitialized = 0;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isEnabled     = 0;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].isCombo       = 1;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pData         = ( void * )&ACCELERO_Data[ LSM303AGR_X_0 ];
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pVTable       = ( void * )&LSM303AGR_X_Drv;
//	ACCELERO_SensorHandle[ LSM303AGR_X_0 ].pExtVTable    = 0;
//
//	LSM303AGR_X_0_Data.comboData = &LSM303AGR_Combo_Data[0];
//	ACCELERO_Data[ LSM303AGR_X_0 ].pComponentData = ( void * )&LSM303AGR_X_0_Data;
//	ACCELERO_Data[ LSM303AGR_X_0 ].pExtData       = 0;
//
//	Sensor_IO_SPI_CS_Init((void *)&ACCELERO_SensorHandle[ LSM303AGR_X_0 ]);
//
//	if(LSM303AGR_Combo_Data[0].isMagInitialized == 0)
//	{
//	// SPI Serial Interface Mode selection --> 3Wires
//	if( Sensor_IO_Write((void *)&ACCELERO_SensorHandle[ LSM303AGR_X_0 ], LSM303AGR_ACC_CTRL_REG4, &data, 1) )
//	{
//	  return COMPONENT_ERROR;
//	}
//	}


	Add_ConfigW2ST_Service();
	Add_HWServW2ST_Service(); // service only gor one sensor, FIXME

	timer_start();

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// MASTER

	master_threadId = 					osThreadCreate(		osThread(	THREAD_MASTER), 		NULL);
	master_queueId = 					osMessageCreate(	osMessageQ(	MESSAGE_MASTER), 		NULL);
	delayed_queueId = 					osMessageCreate(	osMessageQ(	MESSAGE_DELAYED), 		NULL);

	timer_period_master =				INIT_MASTER_TIMER_PERIOD;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// BLE

	ble_threadId = 						osThreadCreate(		osThread(	THREAD_BLE), 		NULL);
	ble_semId = 						osSemaphoreCreate(	osSemaphore(SEMAPHORE_BLE), 	1);

	timer_period_ble =					INIT_BLE_TIMER_PERIOD;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// ALL SENSORS

	thread_poolId = 					osPoolCreate(		osPool(		thread_pool));
	message_poolId = 					osPoolCreate(		osPool(		MESSAGE_SENSOR));

	mutex_semId = 						osSemaphoreCreate(	osSemaphore(SEMAPHORE_MUTEX), 		1);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SENSOR_0

	sprintf(sensorSetup[SENSOR_0].name,				"Arm sensor");
	sprintf(sensorSetup[SENSOR_0].code,				"...");
	sensorSetup[SENSOR_0].data_type =				AXIS; // INT16;
	sensorSetup[SENSOR_0].data_len = 				1;
	sensorSetup[SENSOR_0].sample_packing = 			1;
	sensorSetup[SENSOR_0].sensor_type =				DATA_GENERIG;


//	sensorSetup[SENSOR_0].en_threads[0] =			1;
//	sensorSetup[SENSOR_0].en_threads[1] =			1;
//	sensorSetup[SENSOR_0].en_threads[2] =			1;
//	sensorSetup[SENSOR_0].en_threads[3] =			1;
//	sensorSetup[SENSOR_0].en_threads[4] =			1;

	sensorSetup[SENSOR_0].get_threadId =			osThreadCreate(		osThread(	THREAD_GET_SENSOR_0		), 	NULL);
	sensorSetup[SENSOR_0].proc1_threadId =			osThreadCreate(		osThread(	THREAD_PROC1_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].proc2_threadId =			osThreadCreate(		osThread(	THREAD_PROC2_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].thrsh_threadId =			osThreadCreate(		osThread(	THREAD_THRSH_SENSOR_0	), 	NULL);
	sensorSetup[SENSOR_0].send_threadId =			osThreadCreate(		osThread(	THREAD_SEND_SENSOR_0	), 	NULL);

	sensorSetup[SENSOR_0].proc1_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_PROC1_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].proc2_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_PROC2_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].thrsh_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_THRSH_SENSOR_0	), NULL);
	sensorSetup[SENSOR_0].send_queueId = 			osMessageCreate(	osMessageQ(	MESSAGE_SEND_SENSOR_0	), NULL);

//	sensorSetup[SENSOR_0].outGetQueueId[1] = 		sensorSetup[SENSOR_0].proc1_queueId;
//	sensorSetup[SENSOR_0].outGetQueueId[2] = 		sensorSetup[SENSOR_0].proc2_queueId;
//	sensorSetup[SENSOR_0].outGetQueueId[3] = 		sensorSetup[SENSOR_0].thrsh_queueId;
//	sensorSetup[SENSOR_0].outGetQueueId[4] = 		sensorSetup[SENSOR_0].send_queueId;

	sensorSetup[SENSOR_0].get_semId = 				osSemaphoreCreate(	osSemaphore(SEMAPHORE_SENSOR_0), 	1);

	sensorSetup[SENSOR_0].samp_timer_Callback = 	samp_timer_0_Callback;
	osTimerDef( 									TIMER_SAMP_SENSOR_0, sensorSetup[SENSOR_0].samp_timer_Callback);
	sensorSetup[SENSOR_0].samp_timerId = 			osTimerCreate(osTimer(TIMER_SAMP_SENSOR_0), osTimerPeriodic, &exec);

	sensorSetup[SENSOR_0].send_timer_Callback = 	send_timer_0_Callback;
	osTimerDef( 									TIMER_SEND_SENSOR_0, sensorSetup[SENSOR_0].send_timer_Callback);
	sensorSetup[SENSOR_0].send_timerId = 			osTimerCreate(osTimer(TIMER_SEND_SENSOR_0), osTimerPeriodic, &exec);

	sensorSetup[SENSOR_0].last_value = 				(SensorsData *) pvPortMalloc(sizeof(SensorsData));
	sensorSetup[SENSOR_0].last_value->message_id = 	READ_LAST_VALUE;
	sensorSetup[SENSOR_0].last_value->data_len =	sensorSetup[SENSOR_0].data_len;
	sensorSetup[SENSOR_0].last_value->data = 		dataMalloc(sensorSetup[SENSOR_0].data_type, sensorSetup[SENSOR_0].data_len);

//	sensorSetup[SENSOR_0].charHandle = 				addSensorCharacteristc(SENSOR_0, sensorSetup[SENSOR_0].sensor_type); // generic charHandle, DELETEME

	sensorSetup[SENSOR_0].charHandleRaw = 			addSensorCharacteristc(SENSOR_0, 0);
	sensorSetup[SENSOR_0].charHandleProcessing1 = 	addSensorCharacteristc(SENSOR_0, 1);
	sensorSetup[SENSOR_0].charHandleProcessing2 = 	addSensorCharacteristc(SENSOR_0, 2);

	sensorSetup[SENSOR_0].data_threshold = 			data_threshold;

	sensorSetup[SENSOR_0].get_data = 				BSP_ACCELERO_Get_Axes;

	sensorSetup[SENSOR_0].samp_period_ms =			RAW_SAMP_PERIOD_VALUE;
	sensorSetup[SENSOR_0].send_period_ms =			PROC_SEND_PERIOD_VALUE;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//	#define TESTCODE1
//	#define TESTCODE2

#ifdef TESTCODE1
	for(;;)
	{
		  LED_ON;
		  HAL_Delay(1000);
		  LED_OFF;
		  HAL_Delay(1000);
	}
#endif

#ifdef TESTCODE2

	STLBLE_PRINTF("\r\nTEST code");

	input = input_R;



	timer_0 = timer_get();

	conv1_input = input; conv1_output = cnn_a_buffer;
	arm_convolve_HWC_q7_basic_nonsquare_div(conv1_input, CONV1_IN_DIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_K_DIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_SCALE, conv1_output, CONV1_OUT_DIM, 1, col_buffer, NULL);

	timer_1 = timer_get();

	relu1_inout = cnn_a_buffer;
	arm_relu_q7(relu1_inout, RELU1_IN_DIM);

	timer_2 = timer_get();

	maxpool1_inout = cnn_a_buffer;
	arm_maxpool_q7_HWC_1D(maxpool1_inout, POOL1_IN_DIM, POOL1_IF, POOL1_K_DIM, POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM);

	timer_3 = timer_get();

	conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer;

#if defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
#endif
#if defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_basic_nonsquare_div(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_fast_nonsquare(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
#endif
#if !defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_convolve_HWC_q7_fast_nonsquare_div(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
#endif

	timer_4 = timer_get();

	relu2_inout = cnn_b_buffer;
	arm_relu_q7(relu2_inout, RELU2_IN_DIM);

	timer_5 = timer_get();

	maxpool2_inout = cnn_b_buffer;
	arm_maxpool_q7_HWC_1D(maxpool2_inout, POOL2_IN_DIM, POOL2_IF, POOL2_K_DIM, POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM);

	timer_6 = timer_get();

	fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
#if defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_div(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
#endif
#if !defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt_div(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
#endif

	timer_7 = timer_get();

	relu3_inout = cnn_a_buffer;
	arm_relu_q7(relu3_inout, RELU3_IN_DIM);

	timer_8 = timer_get();

	fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
#if defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_div(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
#endif
#if !defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
	arm_fully_connected_q7_opt_div(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
#endif

	timer_9 = timer_get();



	STLBLE_PRINTF("\r\n%d", timer_1 - timer_0 - 1); // CONV1 time:
	STLBLE_PRINTF("\r\n%d", timer_2 - timer_1 - 1); // RELU1 time:
	STLBLE_PRINTF("\r\n%d", timer_3 - timer_2 - 1); // POOL1 time:
	STLBLE_PRINTF("\r\n%d", timer_4 - timer_3 - 1); // CONV2 time:
	STLBLE_PRINTF("\r\n%d", timer_5 - timer_4 - 1); // RELU2 time:
	STLBLE_PRINTF("\r\n%d", timer_6 - timer_5 - 1); // POOL2 time:
	STLBLE_PRINTF("\r\n%d", timer_7 - timer_6 - 1); // FULL1 time:
	STLBLE_PRINTF("\r\n%d", timer_8 - timer_7 - 1); // RELU3 time:
	STLBLE_PRINTF("\r\n%d", timer_9 - timer_8 - 1); // FULL2 time:

	STLBLE_PRINTF("\r\n\nDIV NN time:   %d", timer_9 - timer_0 - 9);

	for (;;)
#endif



	osKernelStart();					// Start scheduler

	for (;;);							// We should never get here as control is now taken by the scheduler
}

static void master_thread(void const *argument)
{
	(void) argument;

	Sensor_IO_SPI_CS_Init_All();		// Configure and disable all the Chip Select pins

	for (int i = 0; i < NUMBER_OF_SENSORS; i++)
	{
//		if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
//		if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
//		if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
//		if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
//		if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
//
////		sensorSetup[i].samp_period_ms = 50;
//////		osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
////		osTimerStop( sensorSetup[i].samp_timerId);
////		osTimerStop( sensorSetup[i].send_timerId);

		sensorSetup[i].out_get_proc1_queueId =	sensorSetup[i].proc1_queueId;
		sensorSetup[i].out_get_proc2_queueId =	sensorSetup[i].proc2_queueId;
		sensorSetup[i].out_get_send_queueId = 	sensorSetup[i].send_queueId;
		sensorSetup[i].out_proc1_queueId = 		sensorSetup[i].proc2_queueId;
		sensorSetup[i].out_proc2_queueId = 		sensorSetup[i].send_queueId;
		sensorSetup[i].out_thrsh_queueId = 		sensorSetup[i].send_queueId;

////		sensorSetup[i].sample_packing = 1;
//		if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
//		if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
//		if(sensorSetup[i].en_threads[2]) osThreadResume(sensorSetup[i].proc2_threadId);
//		if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
//		if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
	}

	osEvent evt;
	int *message;

	init_all_task();

	master_timer_Start();
	ble_timer_Start();

	timer_workload = HAL_GetTick();



	for (;;)
	{
		evt = osMessageGet(master_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			message = (int *) evt.value.p;

			switch (*message)
			{
//				case TASK_SETUP:
//
//					for (int i = 0; i < NUMBER_OF_SENSORS; i++)
//					{
//
//						switch ((taskState & (3 << (2*i))) >> (2*i))
//						{
//							case (0):
//								osTimerStop( sensorSetup[i].samp_timerId);
//								osTimerStop( sensorSetup[i].send_timerId);
//
////								if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
//							break;
//
//							case (1):
//								sensorSetup[i].samp_period_ms = 10;
//								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
//								osTimerStop( sensorSetup[i].send_timerId);
//
//								sensorSetup[i].out_get_queueId = 	sensorSetup[i].thrsh_queueId;
////								sensorSetup[i].out_get_queueId = 	sensorSetup[i].send_queueId;
//								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//
//								if (i != SENSOR_0) sensorSetup[i].sample_packing = 1;
//								if (i == SENSOR_0) sensorSetup[i].sample_packing = 1; // 8;
//
////								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
//							break;
//
//							case (2):
////								powerControl(RUN, 8);
//
//								sensorSetup[i].samp_period_ms = 20;
//								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
////								if (i == SENSOR_0) osTimerStart(sensorSetup[i].send_timerId, sensorSetup[i].send_period_ms);
//
//								sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
//								sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].send_queueId;
////								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//
//								sensorSetup[i].sample_packing = 1;
//
////								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
//
//							break;
//
//							case (3):
//								sensorSetup[i].samp_period_ms = 50;
//								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
//								osTimerStop( sensorSetup[i].send_timerId);
//
//								#ifdef ARMVERSION
//								sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
//								sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].proc2_queueId;
//								sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].send_queueId;
//								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//								#endif
//
//								#ifdef DIRECTIONSVERSION
//								sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
//								sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].proc2_queueId;
//								sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].send_queueId;
//								sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//								#endif
//
//								sensorSetup[i].sample_packing = 1;
//
////								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadResume(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
//							break;
//
//							default:
//								osTimerStop( sensorSetup[i].samp_timerId);
//								osTimerStop( sensorSetup[i].send_timerId);
//
////								if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
//							break;
//						}
//					}
//
//					break;

			case TASK_SETUP:
				STLBLE_PRINTF("TASK\r\n");

				for (int i = 0; i < NUMBER_OF_SENSORS; i++)
				{

					switch (taskState)
					{
						case (0):
//							STLBLE_PRINTF("MIAO\r\n");
							osTimerStop( sensorSetup[i].samp_timerId);
//							osTimerStop( sensorSetup[i].send_timerId);
							sensorSetup[i].samp_period_ms = 0;
							break;

						case (1): // raw
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	0;
							sensorSetup[i].out_get_proc2_queueId = 	0;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (2): // mov
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}


							sensorSetup[i].out_get_proc1_queueId = 	sensorSetup[i].proc1_queueId;
							sensorSetup[i].out_get_proc2_queueId = 	0;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	0;

//							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
							sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (3): // mov raw
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	sensorSetup[i].proc1_queueId;
							sensorSetup[i].out_get_proc2_queueId = 	0;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (4): // cnn
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	0;
							sensorSetup[i].out_get_proc2_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	0;

//							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (5): // cnn raw
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	0;
							sensorSetup[i].out_get_proc2_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	sensorSetup[i].send_queueId;

//							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (6): // cnn mov
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	sensorSetup[i].proc1_queueId;
							sensorSetup[i].out_get_proc2_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	0;

//							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

						case (7): // cnn mov raw
							if (sensorSetup[i].samp_period_ms == 0)
							{
								sensorSetup[i].samp_period_ms = 10;
								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
	//							osTimerStop( sensorSetup[i].send_timerId);
							}

							sensorSetup[i].out_get_proc1_queueId = 	sensorSetup[i].proc1_queueId;
							sensorSetup[i].out_get_proc2_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_get_thrsh_queueId = 	0;
							sensorSetup[i].out_get_send_queueId = 	sensorSetup[i].send_queueId;

//							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;

							sensorSetup[i].sample_packing = 1;
							break;

//						case (5): // raw mov
//							if (sensorSetup[i].samp_period_ms == 0)
//							{
//								sensorSetup[i].samp_period_ms = 10;
//								osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
//	//							osTimerStop( sensorSetup[i].send_timerId);
//							}
//
////							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].thrsh_queueId;
//							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//
//							sensorSetup[i].sample_packing = 1;
//						break;
//
//
//
//
//
//
//						case (6):
//							sensorSetup[i].samp_period_ms = 50;
//							osTimerStart(sensorSetup[i].samp_timerId, sensorSetup[i].samp_period_ms);
//							osTimerStop( sensorSetup[i].send_timerId);
//
//							#ifdef ARMVERSION
////							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].send_queueId;
//							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//							#endif
//
//							#ifdef DIRECTIONSVERSION
////							sensorSetup[i].out_get_queueId = 	sensorSetup[i].proc1_queueId;
//							sensorSetup[i].out_proc1_queueId = 	sensorSetup[i].proc2_queueId;
//							sensorSetup[i].out_proc2_queueId = 	sensorSetup[i].send_queueId;
//							sensorSetup[i].out_thrsh_queueId = 	sensorSetup[i].send_queueId;
//							#endif
//
//							sensorSetup[i].sample_packing = 1;
//
////								if(sensorSetup[i].en_threads[0]) osThreadResume(sensorSetup[i].get_threadId);
////								if(sensorSetup[i].en_threads[1]) osThreadResume(sensorSetup[i].proc1_threadId);
////								if(sensorSetup[i].en_threads[2]) osThreadResume(sensorSetup[i].proc2_threadId);
////								if(sensorSetup[i].en_threads[3]) osThreadResume(sensorSetup[i].thrsh_threadId);
////								if(sensorSetup[i].en_threads[4]) osThreadResume(sensorSetup[i].send_threadId);
//						break;

						default:
							osTimerStop( sensorSetup[i].samp_timerId);
							osTimerStop( sensorSetup[i].send_timerId);

//								if(sensorSetup[i].en_threads[0]) osThreadSuspend(sensorSetup[i].get_threadId);
//								if(sensorSetup[i].en_threads[1]) osThreadSuspend(sensorSetup[i].proc1_threadId);
//								if(sensorSetup[i].en_threads[2]) osThreadSuspend(sensorSetup[i].proc2_threadId);
//								if(sensorSetup[i].en_threads[3]) osThreadSuspend(sensorSetup[i].thrsh_threadId);
//								if(sensorSetup[i].en_threads[4]) osThreadSuspend(sensorSetup[i].send_threadId);
							break;
					}
				}

				break;

				case SAMP_PERIOD_SETUP:

					sensorSetup[samp_period_ms_index].samp_period_ms = samp_period_ms_value;
					init_all_task();

					break;

				case SEND_PERIOD_SETUP:

					sensorSetup[send_period_ms_index].send_period_ms = send_period_ms_value;
					init_all_task();

					break;

				case PERIOD_CHECK:

//					if(0) GetBatteryInfoData();

					workLoad = (HAL_GetTick() - timer_workload) / (timer_period_master / (float) (MAX_WORKLOAD));
					timer_workload = HAL_GetTick();
//					STLBLE_PRINTF("\r\n\r\nWorkload: %d", workLoad);

					break;

				default:

					// ...

					break;

			}

			osPoolFree(thread_poolId, message);
		}
	}
}

static void ble_thread(	void const *argument)
{

	uint32_t StartTime;

	StartTime = osKernelSysTick();

	for (;;)
	{
		osSemaphoreWait(ble_semId, osWaitForever);

		if (!connected)
		{
			if (!TargetBoardFeatures.LedStatus)
			{
				if (osKernelSysTick()-StartTime > (float)1000)
				{
					StartTime = osKernelSysTick();
				}
			}
			else
			{
				if (osKernelSysTick()-StartTime > (float)100)
				{
					StartTime = osKernelSysTick();
				}
			}
		}

		if (HCI_ProcessEvent)							// Handle BLE event
		{
			HCI_ProcessEvent=0;
			HCI_Process();
		}

		if (set_connectable)							// Update the BLE advertise data and make the Board connectable
		{
			setConnectable();
			set_connectable = FALSE;
		}

//	    __WFI();
	}
}









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0

	static void get_thread_0(	void const *argument) {	gp_read_from_sensor(	SENSOR_0 ); }
	static void proc1_thread_0(	void const *argument) { balance_state_detector(	SENSOR_0 ); }
	static void proc2_thread_0(	void const *argument) { data_classifier(		SENSOR_0 ); }
	static void thrsh_thread_0(	void const *argument) { gp_data_threshold(		SENSOR_0 ); }
	static void send_thread_0(	void const *argument) { gp_send_to_gateway(		SENSOR_0 ); }









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GENERAL PURPOSE FUNCTIONS

static void gp_read_from_sensor(int sensorId)
{
	SensorsData *tempdata;
	SensorsData *mptr;
	uint32_t samp_cnt = 0;

	tempdata = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
	tempdata->data_len = 1;
	tempdata->data = dataMalloc(AXIS, tempdata->data_len);

	#ifdef VERTICAL
	int16_t temp;
	#endif

//	uint32_t StartTime = osKernelSysTick();
//	while (osKernelSysTick()-StartTime < (float)1000);

	for (;;)
	{
		osSemaphoreWait(sensorSetup[sensorId].get_semId, osWaitForever);

		if ( sensorSetup[sensorId].get_data(LSM303AGR_X_0_handle, ((SensorAxes_t*)(tempdata->data))) == COMPONENT_ERROR ) // LSM6DSM_G_0_handle
		{
			((SensorAxes_t*)(tempdata->data))->AXIS_X = -1;
			((SensorAxes_t*)(tempdata->data))->AXIS_Y = -1;
			((SensorAxes_t*)(tempdata->data))->AXIS_Z = -1;
		}
		tempdata->ms_counter = osKernelSysTick();
		tempdata->message_id = READ_FROM_FIFO;
		samp_cnt += 1;

//		STLBLE_PRINTF("ACC X: %d\t\t", 		((SensorAxes_t*)(tempdata->data))->AXIS_X);
////		STLBLE_PRINTF("GYR X: %d\r\n", 		((SensorAxes_t*)(mptr->data)+1)->AXIS_X);
//
//		STLBLE_PRINTF("ACC Y: %d\t\t", 		((SensorAxes_t*)(tempdata->data))->AXIS_Y);
////		STLBLE_PRINTF("GYR Y: %d\r\n", 		((SensorAxes_t*)(mptr->data)+1)->AXIS_Y);
//
//		STLBLE_PRINTF("ACC Z: %d\t\t", 		((SensorAxes_t*)(tempdata->data))->AXIS_Z);
////		STLBLE_PRINTF("GYR Z: %d\r\n\n", 	((SensorAxes_t*)(mptr->data)+1)->AXIS_Z);

		if (sensorSetup[sensorId].out_get_proc1_queueId != 0 && !(samp_cnt % PROC1DOWNSAMPLE)){
			mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
			mptr->data_len = 1;
			mptr->data = dataMalloc(AXIS, mptr->data_len);
			((SensorAxes_t*)(mptr->data))->AXIS_X = ((SensorAxes_t*)(tempdata->data))->AXIS_X;
			((SensorAxes_t*)(mptr->data))->AXIS_Y = ((SensorAxes_t*)(tempdata->data))->AXIS_Y;
			((SensorAxes_t*)(mptr->data))->AXIS_Z = ((SensorAxes_t*)(tempdata->data))->AXIS_Z;
			mptr->ms_counter = tempdata->ms_counter = osKernelSysTick();
			mptr->message_id = tempdata->message_id = READ_FROM_FIFO;
			if (osMessagePut(sensorSetup[sensorId].out_get_proc1_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
		}

		if (sensorSetup[sensorId].out_get_proc2_queueId != 0 && !(samp_cnt % PROC2DOWNSAMPLE)){
			mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
			mptr->data_len = 1;
			mptr->data = dataMalloc(AXIS, mptr->data_len);
			((SensorAxes_t*)(mptr->data))->AXIS_X = ((SensorAxes_t*)(tempdata->data))->AXIS_X;
			((SensorAxes_t*)(mptr->data))->AXIS_Y = ((SensorAxes_t*)(tempdata->data))->AXIS_Y;
			((SensorAxes_t*)(mptr->data))->AXIS_Z = ((SensorAxes_t*)(tempdata->data))->AXIS_Z;
			mptr->ms_counter = tempdata->ms_counter = osKernelSysTick();
			mptr->message_id = tempdata->message_id = READ_FROM_FIFO;
			if (osMessagePut(sensorSetup[sensorId].out_get_proc2_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
		}

		if (sensorSetup[sensorId].out_get_send_queueId != 0){
			mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
			mptr->data_len = 1;
			mptr->data = dataMalloc(AXIS, mptr->data_len);
			((SensorAxes_t*)(mptr->data))->AXIS_X = ((SensorAxes_t*)(tempdata->data))->AXIS_X;
			((SensorAxes_t*)(mptr->data))->AXIS_Y = ((SensorAxes_t*)(tempdata->data))->AXIS_Y;
			((SensorAxes_t*)(mptr->data))->AXIS_Z = ((SensorAxes_t*)(tempdata->data))->AXIS_Z;
			mptr->ms_counter = tempdata->ms_counter = osKernelSysTick();
			mptr->message_id = tempdata->message_id = READ_FROM_FIFO;
		    mptr->charHandle = sensorSetup[sensorId].charHandleRaw;
			if (osMessagePut(sensorSetup[sensorId].out_get_send_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
		}

//		mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
////		mptr->data_len = sensorSetup[sensorId].data_len;
//		mptr->data_len = 1;
//		//    mptr->data = dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
//		//    mptr->dataraw = dataMalloc(AXISRAW, mptr->data_len);
//		    mptr->data = dataMalloc(AXIS, mptr->data_len);
//		    mptr->charHandle = sensorSetup[sensorId].charHandleRaw;
//
//		    if (mptr != NULL)
//		    {
//				if ( sensorSetup[sensorId].get_data(LSM303AGR_X_0_handle, ((SensorAxes_t*)(mptr->data))) == COMPONENT_ERROR ) // LSM6DSM_G_0_handle
//				{
//				((SensorAxes_t*)(mptr->data))->AXIS_X = -1;
//				((SensorAxes_t*)(mptr->data))->AXIS_Y = -1;
//				((SensorAxes_t*)(mptr->data))->AXIS_Z = -1;
//				}
//
//				#ifdef M0DEV
//				// calibration, FIXME
//				((SensorAxes_t*)(mptr->data))->AXIS_X += 26;
//				((SensorAxes_t*)(mptr->data))->AXIS_Y += 62;
//				//			((SensorAxes_t*)(mptr->data))->AXIS_Z = -1;
//				#endif
//
//				#ifdef M2
//				// calibration, FIXME
//				((SensorAxes_t*)(mptr->data))->AXIS_X += -4;
//				((SensorAxes_t*)(mptr->data))->AXIS_Y += 10;
//				//			((SensorAxes_t*)(mptr->data))->AXIS_Z = -1;
//				#endif
//
//				#ifdef VERTICAL
//				temp = ((SensorAxes_t*)(mptr->data))->AXIS_X;
//				((SensorAxes_t*)(mptr->data))->AXIS_X = -((SensorAxes_t*)(mptr->data))->AXIS_Y;
//				((SensorAxes_t*)(mptr->data))->AXIS_Y = -((SensorAxes_t*)(mptr->data))->AXIS_Z;
//				((SensorAxes_t*)(mptr->data))->AXIS_Z = -temp;
//				#endif
//
////				if ( sensorSetup[sensorId].get_data(LSM6DSM_G_0_handle, ((SensorAxes_t*)(mptr->data)+1)) == COMPONENT_ERROR ) //
////				{
////			//        ((SensorAxesRaw_t*)(mptr->dataraw))->AXIS_X = -1;
////			//        ((SensorAxesRaw_t*)(mptr->dataraw))->AXIS_Y = -1;
////			//        ((SensorAxesRaw_t*)(mptr->dataraw))->AXIS_Z = -1;
////
////					((SensorAxes_t*)(mptr->data)+1)->AXIS_X = -1;
////					((SensorAxes_t*)(mptr->data)+1)->AXIS_Y = -1;
////					((SensorAxes_t*)(mptr->data)+1)->AXIS_Z = -1;
////				}
//
////				STLBLE_PRINTF("ACC X: %d\t\t", 		((SensorAxes_t*)(mptr->data))->AXIS_X);
////				STLBLE_PRINTF("GYR X: %d\r\n", 		((SensorAxes_t*)(mptr->data)+1)->AXIS_X);
////
////				STLBLE_PRINTF("ACC Y: %d\t\t", 		((SensorAxes_t*)(mptr->data))->AXIS_Y);
////				STLBLE_PRINTF("GYR Y: %d\r\n", 		((SensorAxes_t*)(mptr->data)+1)->AXIS_Y);
////
////				STLBLE_PRINTF("ACC Z: %d\t\t", 		((SensorAxes_t*)(mptr->data))->AXIS_Z);
////				STLBLE_PRINTF("GYR Z: %d\r\n\n", 	((SensorAxes_t*)(mptr->data)+1)->AXIS_Z);
//
//				mptr->ms_counter = osKernelSysTick();
//				mptr->message_id = READ_FROM_FIFO;
//			}
//		if (osMessagePut(sensorSetup[sensorId].out_get_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
////		vPortFree(mptr->dataraw);
////		vPortFree(mptr->data);
////		vPortFree(mptr);
	}
}

static void gp_data_threshold(int sensorId)
{
	osEvent evt;
	SensorsData *mptr;
	SensorsData *rptr;

	int sendCnt = 0;

	for (;;)
	{
		evt = 	osMessageGet(sensorSetup[sensorId].thrsh_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{

			rptr = (SensorsData *) evt.value.p;

			if(!sendCnt)
			{
				mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData));
				mptr->charHandle = 		rptr->charHandle;
				mptr->data_len = 		rptr->data_len * sensorSetup[sensorId].sample_packing;
				mptr->data = 			dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
				sendCnt = 				sensorSetup[sensorId].sample_packing;
			}

			dataCopy(sensorSetup[sensorId].data_type, &sendCnt, rptr, mptr);

			if(!sendCnt)
			{
				if(sensorSetup[sensorId].data_threshold(mptr) == SEND)
				{
					mptr->ms_counter = rptr->ms_counter;
					if (osMessagePut(sensorSetup[sensorId].out_thrsh_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();
				}
			}

		}

		if(rptr->message_id == READ_FROM_FIFO)
		{
			vPortFree(rptr->data);
			vPortFree(rptr);
		}
	}
}

static void gp_send_to_gateway(int sensorId)
{
	osEvent evt;
	SensorsData *rptr;

	unsigned long int debug_cnt = 0;

	for (;;)
	{
		evt = 	osMessageGet(sensorSetup[sensorId].send_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;



//			*((int16_t*) rptr->data) = workLoad;
//			rptr->data_len = 1;
//
//			charUpdate(
//						sensorSetup[sensorId].charHandle,
//						sensorSetup[sensorId].data_type,
//						sensorSetup[sensorId].sensor_type,
//						rptr->ms_counter,
//						rptr->data,
//						rptr->data_len
//					  );



//			STLBLE_PRINTF("\r\n1X: %d", ((SensorAxes_t*)(rptr->data))->AXIS_X);
//			STLBLE_PRINTF("\r\n1Y: %d", ((SensorAxes_t*)(rptr->data))->AXIS_Y);
//			STLBLE_PRINTF("\r\n1Z: %d\n", ((SensorAxes_t*)(rptr->data))->AXIS_Z);
//			STLBLE_PRINTF("\r\n2X: %d", ((SensorAxes_t*)(rptr->data)+1)->AXIS_X);
//			STLBLE_PRINTF("\r\n2Y: %d", ((SensorAxes_t*)(rptr->data)+1)->AXIS_Y);
//			STLBLE_PRINTF("\r\n2Z: %d\n", ((SensorAxes_t*)(rptr->data)+1)->AXIS_Z);
//			STLBLE_PRINTF("\r\n3X: %d", ((SensorAxes_t*)(rptr->data)+2)->AXIS_X);
//			STLBLE_PRINTF("\r\n3Y: %d", ((SensorAxes_t*)(rptr->data)+2)->AXIS_Y);
//			STLBLE_PRINTF("\r\n3Z: %d\n", ((SensorAxes_t*)(rptr->data)+2)->AXIS_Z);
//			STLBLE_PRINTF("\r\n4X: %d", ((SensorAxes_t*)(rptr->data)+3)->AXIS_X);
//			STLBLE_PRINTF("\r\n4Y: %d", ((SensorAxes_t*)(rptr->data)+3)->AXIS_Y);
//			STLBLE_PRINTF("\r\n4Z: %d\n", ((SensorAxes_t*)(rptr->data)+3)->AXIS_Z);

//			STLBLE_PRINTF("\r\n%d", rptr->data_len);




//			if (debug_cnt % 4 == 0) {

				charUpdate(
	//						sensorSetup[sensorId].charHandle,
	//						sensorSetup[sensorId].charHandleRaw,
							rptr->charHandle,
							sensorSetup[sensorId].data_type,
							sensorSetup[sensorId].sensor_type,
							rptr->ms_counter,
							rptr->data,
							rptr->data_len
						  );

//			}
//			debug_cnt++;

		}

		vPortFree(rptr->data);
		vPortFree(rptr);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////









//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SENSOR_0: FUNCTIONS & TIMER

static void balance_state_detector(int sensorId)
{
	osEvent evt;
	SensorsData *mptr;
	SensorsData *rptr;

	uint8_t procDataIdx = 0;
	uint8_t medianIdx = (int8_t) (AVERAGE_SAMPLE / 2);

	float_t pitch = 0;
	float_t roll = 0;

	float_t s_module = 0;
	float_t s_angle = 0;

//	int8_t minRotXrmIdx = 0;
//	int8_t maxRotXrmIdx = 0;
//	int8_t minRotYrmIdx = 0;
//	int8_t maxRotYrmIdx = 0;

//	evt = osMessageGet(sensorSetup[sensorId].proc1_queueId, osWaitForever);
//
//	if (evt.status == osEventMessage)
//	{
//		rptr = (SensorsData *) evt.value.p;
//
//		minRotX[0] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//		maxRotX[0] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//		minRotY[0] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;
//		maxRotY[0] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;
//
//		minRotXrmIdx = 0;
//		maxRotXrmIdx = 0;
//		minRotYrmIdx = 0;
//		maxRotYrmIdx = 0;
//	}
//
//	vPortFree(rptr->data);
//	vPortFree(rptr);

	for (int i = 0; i < AVERAGE_SAMPLE; i++)
	{
		evt = osMessageGet(sensorSetup[sensorId].proc1_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;

//			STLBLE_PRINTF("\r\nBAY %d", i);

			procDataX[i] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
			procDataY[i] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;

//			minRotX[i] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//			maxRotX[i] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//			minRotY[i] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;
//			maxRotY[i] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;

//			if (minRotX[i] > minRotX[minRotXrmIdx]) minRotXrmIdx = i;
//			if (maxRotX[i] < maxRotX[minRotXrmIdx]) maxRotXrmIdx = i;
//			if (minRotY[i] > minRotY[minRotXrmIdx]) minRotYrmIdx = i;
//			if (maxRotY[i] < maxRotY[minRotXrmIdx]) maxRotYrmIdx = i;

			vPortFree(rptr->data);
			vPortFree(rptr);
		}
	}

	for (;;)
	{
		evt = osMessageGet(sensorSetup[sensorId].proc1_queueId, osWaitForever);

		if (evt.status == osEventMessage)
		{
			rptr = (SensorsData *) evt.value.p;

//			STLBLE_PRINTF("\r%d  \t%d      ",((SensorAxes_t*)(rptr->data))->AXIS_X,((SensorAxes_t*)(rptr->data))->AXIS_Y);

			procDataX[procDataIdx % AVERAGE_SAMPLE] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
			procDataY[procDataIdx % AVERAGE_SAMPLE] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;

//			if (((SensorAxes_t*)(rptr->data))->AXIS_X < minRotX[minRotXrmIdx]) minRotX[minRotXrmIdx] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//			if (((SensorAxes_t*)(rptr->data))->AXIS_X > maxRotX[minRotXrmIdx]) maxRotX[maxRotXrmIdx] = ((SensorAxes_t*)(rptr->data))->AXIS_X;
//			if (((SensorAxes_t*)(rptr->data))->AXIS_Y < minRotY[minRotXrmIdx]) minRotY[minRotYrmIdx] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;
//			if (((SensorAxes_t*)(rptr->data))->AXIS_Y > maxRotY[minRotXrmIdx]) maxRotY[maxRotYrmIdx] = ((SensorAxes_t*)(rptr->data))->AXIS_Y;
//
//			minRotXrmIdx = 0;
//			maxRotXrmIdx = 0;
//			minRotYrmIdx = 0;
//			maxRotYrmIdx = 0;

			#ifdef FMODE1
			#endif
			#ifdef FMODE2
				selectionSort(procDataX, AVERAGE_SAMPLE);
				selectionSort(procDataY, AVERAGE_SAMPLE);
			#endif

			int debug_once = true;
			if (debug_once)
			{
			debug_once = false;

			if(procDataIdx % DOWNSAMPLE == 0)
//			if(true)
			{

				mptr = (SensorsData *) pvPortMalloc(sizeof(SensorsData));
				mptr->data_len = 1;
				mptr->data = dataMalloc(AXIS, mptr->data_len);
				mptr->charHandle = sensorSetup[sensorId].charHandleProcessing1;

//				medianIdx = procDataIdx % AVERAGE_SAMPLE;

				#ifdef FMODE1
					pitch = 0;
					roll = 0;
					for (int i = 0; i < AVERAGE_SAMPLE; i++)
					{
						pitch += procDataX[i];
						roll += procDataY[i];
					}
					pitch /= 10.9 * AVERAGE_SAMPLE;
					roll /= 10.9 * AVERAGE_SAMPLE;
				#endif
				#ifdef FMODE2
					pitch = procDataX[medianIdx] / 10.9;
					roll = procDataY[medianIdx] / 10.9;
				#endif


				s_module = sqrt((float_t)(pitch*pitch + roll*roll));
				if (s_module > 90) s_module = 90;
				if (pitch !=0 )
				{
					s_angle = -(atan2(roll,pitch)*180)/3.141592654 + 180;
				}
				else
				{
					s_angle = 0;
				}
//
//				STLBLE_PRINTF("\r\n%d, %d\r\n%f, %f\r\n%f, %f\r\n", procDataX[medianIdx], procDataY[medianIdx], pitch, roll, s_module, s_angle);

				((SensorAxes_t*)(mptr->data))->AXIS_X = (int16_t) s_module;
				((SensorAxes_t*)(mptr->data))->AXIS_Y = (int16_t) s_angle;
//				((SensorAxes_t*)(mptr->data))->AXIS_Z = -1;

//				((SensorAxes_t*)(mptr->data))->AXIS_X = procDataX[medianIdx];
//				((SensorAxes_t*)(mptr->data))->AXIS_Y = procDataY[medianIdx];
////				((SensorAxes_t*)(mptr->data))->AXIS_Z = -1;

				mptr->ms_counter = osKernelSysTick();
				mptr->message_id = READ_FROM_FIFO;

				if (osMessagePut(sensorSetup[sensorId].out_proc1_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();

//				vPortFree(mptr->data);
//				vPortFree(mptr);

//				STLBLE_PRINTF("\r\n%d, %d\r\n", ((SensorAxes_t*)(mptr->data))->AXIS_X, ((SensorAxes_t*)(mptr->data))->AXIS_Y);
//				STLBLE_PRINTF("\r\n%d, %d, %d\r\n", procDataIdx % AVERAGE_SAMPLE, ((SensorAxes_t*)(mptr->data))->AXIS_X, ((SensorAxes_t*)(mptr->data))->AXIS_Y);
			}

//			for (int i = 1; i < AVERAGE_SAMPLE; i++)
//			{
//				if (minRotX[i] > minRotX[minRotXrmIdx]) minRotXrmIdx = i;
//				if (maxRotX[i] < maxRotX[minRotXrmIdx]) maxRotXrmIdx = i;
//				if (minRotY[i] > minRotY[minRotXrmIdx]) minRotYrmIdx = i;
//				if (maxRotY[i] < maxRotY[minRotXrmIdx]) maxRotYrmIdx = i;
//			}

			procDataIdx++;

			}

			vPortFree(rptr->data);
			vPortFree(rptr);
		}
	}
}

static void data_classifier(int sensorId)
{
	osEvent evt;
	SensorsData *rptr;
	SensorsData *mptr;

	uint32_t cnn_input_idx = 0;
	uint32_t raw_input_idx = 0;

	#ifdef DIRECTIONSVERSION
	uint8_t bb_state = 0;
	uint8_t bb_direction = 0;
	#endif

	#ifdef DUMMYINPUT
	uint32_t once = 1;
	#endif

	STLBLE_PRINTF("INIT\r\n");



//	#define SIMPLETEST

#ifdef SIMPLETEST
	/*
	 * 	--- EXAMPLE: CONV2D LAYER ---
	 *
	 * 	IN_DIM = 	10
	 * 	OUT_DIM = 	7
	 * 	WT_DIM = 	4
	 * 	IF = 		2
	 * 	OF = 		2
	 *
	 *	IF1	IF2		K1_IF1	K1_IF2	K2_IF1	K2_IF2		B1	B2		OF1		OF2
	 * 	0	0		1		10		2		20			0	0		0606	1212
	 * 	1	10		1		10		2		20						1010	2020
	 * 	2	20		1		10		2		20						1414	2828
	 * 	3	30		1		10		2		20						1818	3636
	 * 	4	40														2222	4444
	 * 	5	50														2626	5252
	 * 	6	60														3030	6060
	 * 	7	70
	 * 	8	80
	 * 	9	90
	 *
	 *
	 *
	 * 	--- CMSIS INPUT ---
	 *
	 *	input	[IN_DIM * IF] = 		{0,	 0,  1, 10,  2, 20,  3, 30,  4,  40,  5, 50,  6, 60,  7, 70,  8, 80,  9, 90};
	 * 	weights	[WT_DIM * IF * OF] = 	{1, 10,  1, 10,  1, 10,  1, 10,  2,  20,  2, 20,  2, 20,  2, 20};
	 * 	bias	[OF] = 					{0};
	 * 	output	[OUT_DIM * OF] = 		{0};
	 * 	stride = 						1
	 * 	padding = 						0
	 * 	bias_shift = 					0
	 * 	output_shift = 					0
	 *
	 *
	 *
	 * 	--- CMSIS OUTPUT ---
	 *
	 * 	output = {606, 1212, 1010, 2020, 1414, 2828, 1818, 3636, 2222, 4444, 2626, 5252, 3030, 6060}
	 */



	#define TEST__IN_DIM			10
	#define TEST__WTDIM			4
	#define TEST__IF			2
	#define TEST__OF			2
	#define TEST__STRIDE		1
	#define	TEST__PADDING		0
	#define TEST__OUT_DIM		(TEST__IN_DIM-TEST__WTDIM+1)	// with stride equal to 1 and padding equal to zero
	#define TEST__OUTRSHIFT		0
	#define TEST__BIASLSHIFT	0


	q7_t 	test__data	[TEST__IN_DIM*TEST__IF] = 			{0, 0, 1, 10, 2, 20, 3, 30, 4, 40, 5, 50, 6, 60, 7, 70, 8, 80, 9, 90};
	q7_t 	test__wt	[TEST__WTDIM*TEST__IF*TEST__OF] = 	{1, 10, 1, 10, 1, 10, 1, 10, 2, 20, 2, 20, 2, 20, 2, 20};
	q7_t 	test__bias	[TEST__OF] = 						{0};
	int32_t out__test	[TEST__OUT_DIM*TEST__OF] = 			{0};

	q7_t* test__buffer;
	q7_t* test__col_buffer;
	q7_t test__scratch_pad[TEST__OF*TEST__IN_DIM + 2*TEST__IF*TEST__WTDIM + TEST__IN_DIM*TEST__IF] = {0};

	test__buffer = test__scratch_pad;
	test__col_buffer = test__buffer + TEST__OF*TEST__IN_DIM;

	arm_convolve_HWC_q7_basic_nonsquare_o32(				(q7_t*)test__data,
															TEST__IN_DIM, 1,
															TEST__IF,
															test__wt,
															TEST__OF,
															TEST__WTDIM, 1,
															TEST__PADDING, 0,
															TEST__STRIDE, 1,
															test__bias,
															TEST__BIASLSHIFT,
															TEST__OUTRSHIFT,
															test__buffer,
															TEST__OUT_DIM, 1,
															(q15_t*)test__col_buffer, NULL,
															out__test);

	for(int i = 0; i < TEST__OUT_DIM*TEST__OF; i+=TEST__OF)
	{
		for(int j = 0; j < TEST__OF; j++)
		{
			STLBLE_PRINTF("%d, ", out__test[i+j]);
		}
		STLBLE_PRINTF("\r\n");
	}

	while(1);
#endif



//#ifdef OUTPUT_32BIT
//	q31_t 		buffer_o32[CONV1_OF*CONV1_OUT_DIM];
//#endif
//
//	evt = osMessageGet(sensorSetup[sensorId].proc2_queueId, osWaitForever);
//
//	if (evt.status == osEventMessage)
//	{
//		rptr = (SensorsData *) evt.value.p;
//
////		for(int i = 0; i < AVERAGE_SAMPLE; i++) STLBLE_PRINTF("%d ", minRotX[i]);
////		STLBLE_PRINTF("\r\n");
////		for(int i = 0; i < AVERAGE_SAMPLE; i++) STLBLE_PRINTF("%d ", maxRotX[i]);
////		STLBLE_PRINTF("\r\n");
////		for(int i = 0; i < AVERAGE_SAMPLE; i++) STLBLE_PRINTF("%d ", minRotY[i]);
////		STLBLE_PRINTF("\r\n");
////		for(int i = 0; i < AVERAGE_SAMPLE; i++) STLBLE_PRINTF("%d ", maxRotY[i]);
////		STLBLE_PRINTF("\r\n");
//	}
//
//	vPortFree(rptr->data);
//	vPortFree(rptr);

	for (;;)
	{
		evt = osMessageGet(sensorSetup[sensorId].proc2_queueId, osWaitForever);
		LED_ON;

		if (evt.status == osEventMessage)
		{

			rptr = (SensorsData *) evt.value.p;



			#ifndef DIRECTIONSVERSION

			if (true) // (raw_input_idx % CNN_DOWNSCALING == 0)
			{
				#ifndef DUMMYINPUT
				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_X) / QUANT_SCALE, 8);
				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + 1] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_Y) / QUANT_SCALE, 8);
//				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + 2] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_Z) / QUANT_SCALE, 8);
				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + CONV1_IN_DIM * CONV1_IF] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_X) / QUANT_SCALE, 8);
				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + CONV1_IN_DIM * CONV1_IF + 1] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_Y) / QUANT_SCALE, 8);
//				cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + CONV1_IN_DIM * CONV1_IF + 2] = (q7_t) __SSAT((((SensorAxes_t*)(rptr->data))->AXIS_Z) / QUANT_SCALE, 8);
				#endif
	//			STLBLE_PRINTF("\r\n CIAO %d", cnn_input_idx);
	//			STLBLE_PRINTF("\r\n CIAO %d, %d, %d,%d\r\n       ",
	//					cnn_input_idx,
	//					cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF],
	//					cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + 1],
	//					cnn_input[(cnn_input_idx % CONV1_IN_DIM) * CONV1_IF + 2]);

	//			if (debug_once)
				if ((cnn_input_idx % PROC2STEP == 0) && (cnn_input_idx >= CONV1_IN_DIM)) // 25
				{
	//				debug_once = false;

					#ifndef DUMMYINPUT
	//				input = (q7_t*) rptr->data;
	//				input = (q7_t*) cnn_input;
					input = (q7_t*) &cnn_input[((cnn_input_idx % CONV1_IN_DIM) * CONV1_IF) + CONV1_IF];
					#endif

		//#ifdef LOG_PRINT
					timer_0 = timer_get();
		//#endif
		//
		//#ifdef OUTPUT_SHIFT_OLD
		//			conv1_input = input; conv1_output = cnn_a_buffer;
		//			arm_convolve_HWC_q7_basic_nonsquare(conv1_input, CONV1_IN_DIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_K_DIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_RSHIFT, conv1_output, CONV1_OUT_DIM, 1, col_buffer, NULL);
		//
		//			relu1_inout = cnn_a_buffer;
		//			arm_relu_q7(relu1_inout, RELU1_IN_DIM);
		//
		//			maxpool1_inout = cnn_a_buffer;
		//			arm_maxpool_q7_HWC_1D(maxpool1_inout, POOL1_IN_DIM, POOL1_IF, POOL1_K_DIM, POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM);
		//
		//			conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer;
		//			arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
		//
		//			relu2_inout = cnn_b_buffer;
		//			arm_relu_q7(relu2_inout, RELU2_IN_DIM);
		//
		//			maxpool2_inout = cnn_b_buffer;
		//			arm_maxpool_q7_HWC_1D(maxpool2_inout, POOL2_IN_DIM, POOL2_IF, POOL2_K_DIM, POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM);
		//
		//			fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
		//			arm_fully_connected_q7(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
		//
		//			relu3_inout = cnn_a_buffer;
		//			arm_relu_q7(relu3_inout, RELU3_IN_DIM);
		//
		//			fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
		//			arm_fully_connected_q7(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
		//#endif



//					STLBLE_PRINTF("L0\r\n");

					conv1_input = input; conv1_output = cnn_a_buffer; //  * CONV1_IF
					arm_convolve_HWC_q7_basic_nonsquare_div(conv1_input, CONV1_IN_DIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_K_DIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, CONV1_SCALE / ( QUANT_SCALE * CONV1_WEIGHT_SCALE ), conv1_output, CONV1_OUT_DIM, 1, col_buffer, NULL); // / ( QUANT_SCALE * CONV1_WEIGHT_SCALE )

//					#ifdef DUMMYINPUT
//					if (once)
//					{
//						once--;
//						STLBLE_PRINTF("\r\n");
//						for (int i = 0; i < (CONV1_OF); i++)
//						{
//							for (int j = 0; j < (int) (CONV1_OUT_DIM / 1); j++) STLBLE_PRINTF("%d, ", cnn_a_buffer[i + j * CONV1_OF]);
//							STLBLE_PRINTF("\r\n");
//							osDelay(10);
//						}
//					}
//					#endif

					timer_1 = timer_get();
//					STLBLE_PRINTF("L1\r\n");

					relu1_inout = cnn_a_buffer;
					arm_relu_q7(relu1_inout, RELU1_IN_DIM);

					timer_2 = timer_get();
//					STLBLE_PRINTF("L2\r\n");

					maxpool1_inout = cnn_a_buffer;
					arm_maxpool_q7_HWC_1D(maxpool1_inout, POOL1_IN_DIM, POOL1_IF, POOL1_K_DIM, POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM);

//					#ifdef DUMMYINPUT
//					if (once)
//					{
//						STLBLE_PRINTF("\r\n");
//						for (int i = 0; i < (POOL1_IF); i++)
//						{
//							for (int j = 0; j < POOL1_OUT_DIM; j++) STLBLE_PRINTF("%d, ", cnn_b_buffer[i + j * CONV1_OF]);
//							STLBLE_PRINTF("\r\n");
//							osDelay(10);
//						}
//					}
//					#endif

					timer_3 = timer_get();
//					STLBLE_PRINTF("L3\r\n");
					conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer; //  * CONV2_IF

		#if defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
					arm_convolve_HWC_q7_basic_nonsquare(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
		#endif
		#if defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
//					arm_convolve_HWC_q7_basic_nonsquare_div(conv2_input, CONV2_IN_DIM * CONV2_IF, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, ( CONV2_SCALE / ( QUANT_SCALE * CONV2_WEIGHT_SCALE ) ), conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
					arm_convolve_HWC_q7_fast_nonsquare_div(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_SCALE / ( CONV1_SCALE * CONV2_WEIGHT_SCALE ), conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL); //  (QUANT_SCALE / CONV1_SCALE) * CONV2_SCALE / ( QUANT_SCALE * CONV2_WEIGHT_SCALE )
		#endif
		#if !defined(CONV2_BASIC) && defined(OUTPUT_SHIFT)
					arm_convolve_HWC_q7_fast_nonsquare(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_SCALE, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
		#endif
		#if !defined(CONV2_BASIC) && !defined(OUTPUT_SHIFT)
					arm_convolve_HWC_q7_fast_nonsquare_div(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, CONV2_OUT_RSHIFT, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL);
		#endif

//					#ifdef DUMMYINPUT
//					if (once)
//					{
//						once--;
//						STLBLE_PRINTF("\r\n");
//						for (int i = 0; i < (CONV2_OF); i++)
//						{
//							for (int j = 0; j < CONV2_OUT_DIM; j++) STLBLE_PRINTF("%d, ", cnn_b_buffer[i + j * CONV1_OF]);
//							STLBLE_PRINTF("\r\n");
//							osDelay(10);
//						}
//					}
//					#endif

					timer_4 = timer_get();
//					STLBLE_PRINTF("L4\r\n");

					relu2_inout = cnn_b_buffer;
					arm_relu_q7(relu2_inout, RELU2_IN_DIM);

					timer_5 = timer_get();
//					STLBLE_PRINTF("L5\r\n");

					maxpool2_inout = cnn_b_buffer;
					arm_maxpool_q7_HWC_1D(maxpool2_inout, POOL2_IN_DIM, POOL2_IF, POOL2_K_DIM, POOL2_PADDING, POOL2_STRIDE, POOL2_OUT_DIM);

					timer_6 = timer_get();
//					STLBLE_PRINTF("L6\r\n");

					fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer;
		#if defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
					arm_fully_connected_q7(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
		#endif
		#if defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_div(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT,  FC1_SCALE / ( CONV2_SCALE * FC1_PACKED_PARAMS_WEIGHT_SCALE ), fc1_bias, fc1_output, col_buffer);
		#endif
		#if !defined(FULLY1_BASIC) && defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_opt(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer);
		#endif
		#if !defined(FULLY1_BASIC) && !defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_opt_div(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_SCALE, fc1_bias, fc1_output, col_buffer);
		#endif

					timer_7 = timer_get();
//					STLBLE_PRINTF("L7\r\n");

//					#ifdef DUMMYINPUT
//					if (once)
//					{
////						once--;
//						STLBLE_PRINTF("\r\n");
//						for (int i = 0; i < FC1_OUT_DIM; ++i) STLBLE_PRINTF("%d, ", cnn_a_buffer[i]);
//						STLBLE_PRINTF("\r\n");
//					}
//					#endif

					relu3_inout = cnn_a_buffer;
					arm_relu_q7(relu3_inout, RELU3_IN_DIM);

					timer_8 = timer_get();
//					STLBLE_PRINTF("L8\r\n");

					fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer;
		#if defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
					arm_fully_connected_q7(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
		#endif
		#if defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_div(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_SCALE / ( FC1_SCALE * FC2_PACKED_PARAMS_WEIGHT_SCALE ), fc2_bias, fc2_output, col_buffer);
		#endif
		#if !defined(FULLY2_BASIC) && defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_opt(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer);
		#endif
		#if !defined(FULLY2_BASIC) && !defined(OUTPUT_SHIFT)
					arm_fully_connected_q7_opt_div(fc2_input, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_SCALE, fc2_bias, fc2_output, col_buffer);
		#endif

					timer_9 = timer_get();
//					STLBLE_PRINTF("L9\r\n");

					#ifdef DUMMYINPUT
					if (once)
					{
						once--;
						STLBLE_PRINTF("\r\n");
						for (int i = 0; i < FC2_PACKED_PARAMS_BIAS_DIM; ++i) STLBLE_PRINTF("%d, ", cnn_b_buffer[i]);
						STLBLE_PRINTF("\r\n");
					}
					#endif



		//#ifdef OUTPUT_32BIT_OLD
		//			conv1_input = input; conv1_output = cnn_a_buffer; conv1_output_o32 = buffer_o32;
		//			arm_convolve_HWC_q7_basic_nonsquare_o32(conv1_input, CONV1_IN_DIM, 1, CONV1_IF, conv1_wt, CONV1_OF, CONV1_K_DIM, 1, CONV1_PADDING, 0, CONV1_STRIDE, 1, conv1_bias, CONV1_BIAS_LSHIFT, 0, conv1_output, CONV1_OUT_DIM, 1, col_buffer, NULL, conv1_output_o32);
		//			for(int i = 0; i < CONV1_OF*CONV1_OUT_DIM; i++) conv1_output[i] = round(conv1_output_o32[i]/CONV1_SCALE);
		//
		//			relu1_inout = cnn_a_buffer;
		//			arm_relu_q7(relu1_inout, RELU1_IN_DIM);
		//
		//			maxpool1_inout = cnn_a_buffer;
		//			arm_maxpool_q7_HWC_1D(maxpool1_inout, POOL1_IN_DIM, POOL1_IF, POOL1_K_DIM, POOL1_PADDING, POOL1_STRIDE, POOL1_OUT_DIM);
		//
		//			conv2_input = cnn_a_buffer; conv2_output = cnn_b_buffer; conv2_output_o32 = buffer_o32;
		//			arm_convolve_HWC_q7_basic_nonsquare_o32(conv2_input, CONV2_IN_DIM, 1, CONV2_IF, conv2_wt, CONV2_OF, CONV2_K_DIM, 1, CONV2_PADDING, 0, CONV2_STRIDE, 1, conv2_bias, CONV2_BIAS_LSHIFT, 0, conv2_output, CONV2_OUT_DIM, 1, col_buffer, NULL, conv2_output_o32);
		//			for(int i = 0; i < CONV2_OF*CONV2_OUT_DIM; i++) conv2_output[i] = round(conv2_output_o32[i]/CONV2_OUT_SCALE);
		//
		//			relu2_inout = cnn_b_buffer;
		//			arm_relu_q7(relu2_inout, RELU2_IN_DIM);
		//
		//			maxpool2_inout = cnn_b_buffer;
		//			arm_maxpool_q7_HWC_1D(maxpool2_inout, POOL2_IN_DIM,POOL2_IF,POOL2_K_DIM,POOL2_PADDING,POOL2_STRIDE,POOL2_OUT_DIM);
		//
		//			fc1_input = cnn_b_buffer; fc1_output = cnn_a_buffer; fc1_output_o32 = buffer_o32;
		//			arm_fully_connected_q7_o32(fc1_input, fc1_wt, FC1_IN_DIM, FC1_OUT_DIM, FC1_BIAS_LSHIFT, FC1_OUT_RSHIFT, fc1_bias, fc1_output, col_buffer, fc1_output_o32);
		//			for(int i = 0; i < FC1_OUT_DIM; i++) fc1_output[i] = round(fc1_output_o32[i]/FC1_OUT_SCALE);
		//
		//			relu3_inout = cnn_a_buffer;
		//			arm_relu_q7(relu3_inout, RELU3_IN_DIM);
		//
		//			fc2_input = cnn_a_buffer; fc2_output = cnn_b_buffer; fc2_output_o32 = buffer_o32;
		//			arm_fully_connected_q7_o32(fc1_output, fc2_wt, FC2_IN_DIM, FC2_OUT_DIM, FC2_BIAS_LSHIFT, FC2_OUT_RSHIFT, fc2_bias, fc2_output, col_buffer, fc2_output_o32);
		//			for(int i = 0; i < FC2_OUT_DIM; i++) fc2_output[i] = round(fc2_output_o32[i]/FC2_OUT_SCALE);
		//#endif
		//
		//#ifdef LOG_PRINT
//					STLBLE_PRINTF("\r\nCONV1 time:   %d", timer_1 - timer_0 - 1);
//					STLBLE_PRINTF("\r\nRELU1 time:   %d", timer_2 - timer_1 - 1);
//					STLBLE_PRINTF("\r\nPOOL1 time:   %d", timer_3 - timer_2 - 1);
//					STLBLE_PRINTF("\r\nCONV2 time:   %d", timer_4 - timer_3 - 1);
//					STLBLE_PRINTF("\r\nRELU2 time:   %d", timer_5 - timer_4 - 1);
//					STLBLE_PRINTF("\r\nPOOL2 time:   %d", timer_6 - timer_5 - 1);
//					STLBLE_PRINTF("\r\nFULL1 time:   %d", timer_7 - timer_6 - 1);
//					STLBLE_PRINTF("\r\nRELU3 time:   %d", timer_8 - timer_7 - 1);
//					STLBLE_PRINTF("\r\nFULL2 time:   %d", timer_9 - timer_8 - 1);
//
//					STLBLE_PRINTF("\r\n\nDIV NN time:   %d", timer_9 - timer_0 - 9);
//					STLBLE_PRINTF("\r\n\nWorkload:   %d", workLoad);
		//#endif
		//
					fc2_output_maxindex = 0;
					fc2_output_max = fc2_output[fc2_output_maxindex];

					for (int i = 1; i < FC2_OUT_DIM; ++i)
					{
						if (fc2_output[i] > fc2_output_max)
						{
							fc2_output_max = fc2_output[i];
							fc2_output_maxindex = i;
						}
					}
//					if (fc2_output[1] > 0) fc2_output_maxindex = 1;
		//
		//#ifdef LOG_PRINT
		//			STLBLE_PRINTF("\r\nCLASS: %c\r\n", classes[fc2_output_maxindex]);
		//#endif
		//
					mptr = 					(SensorsData *) pvPortMalloc(sizeof(SensorsData));
					mptr->data_len = 		1;
	//				mptr->data = 			dataMalloc(sensorSetup[sensorId].data_type, mptr->data_len);
					mptr->data = 			dataMalloc(AXIS, mptr->data_len);
					mptr->ms_counter =		osKernelSysTick();
					mptr->message_id =		READ_FROM_FIFO;
					mptr->charHandle = 		sensorSetup[sensorId].charHandleProcessing2;
					((SensorAxes_t*)(mptr->data))->AXIS_X = fc2_output_maxindex + 1;
					((SensorAxes_t*)(mptr->data))->AXIS_Y = fc2_output_maxindex + 1;
					((SensorAxes_t*)(mptr->data))->AXIS_Z = fc2_output_maxindex + 1;
	//				*((int16_t*) mptr->data) = fc2_output_maxindex;

					if (osMessagePut(sensorSetup[sensorId].out_proc2_queueId, (uint32_t) mptr, osWaitForever) != osOK) Error_Handler();

//					STLBLE_PRINTF("\r\n%d\t%d\t%d\t%d", cnn_input_idx, cnn_b_buffer[0], cnn_b_buffer[1], cnn_b_buffer[2]);
				}

	//			if (cnn_input_idx == (CONV1_IN_DIM * CONV1_IF) - 1) cnn_input_idx = 0;
	//			else cnn_input_idx++;
				cnn_input_idx++;
			}
//			raw_input_idx++;s

			vPortFree(rptr->data);
			vPortFree(rptr);

			#endif




			#ifdef DIRECTIONSVERSION
			rptr->charHandle = sensorSetup[sensorId].charHandleProcessing2;

			switch (((SensorAxes_t*)(rptr->data))->AXIS_X)
			{
			 	 case 0 ... 6:
				 	 bb_state = 0;
			 	 	 break;
			 	 case 7 ... 28:
				 	 bb_state = 1;
		 	 	 	 break;
			 	 default:
				 	 bb_state = 2;
			 	 	 break;
			}

			bb_direction = (uint8_t) ((((SensorAxes_t*)(rptr->data))->AXIS_Y)/(360.0/DIRECTIONS) + 0.5) % DIRECTIONS;

			((SensorAxes_t*)(rptr->data))->AXIS_X = bb_state;
			((SensorAxes_t*)(rptr->data))->AXIS_Y = bb_direction;

//			STLBLE_PRINTF("\r\n%d, %d", ((SensorAxes_t*)(rptr->data))->AXIS_X, ((SensorAxes_t*)(rptr->data))->AXIS_Y);

			if (osMessagePut(sensorSetup[sensorId].out_proc2_queueId, (uint32_t) rptr, osWaitForever) != osOK) Error_Handler();
			#endif

		}
	}
}

int data_threshold(SensorsData *data)
{
	return SEND;
//	return DO_NOT_SEND;
}



void samp_timer_0_Callback(void const *arg) { osSemaphoreRelease(sensorSetup[SENSOR_0].get_semId); }

void send_timer_0_Callback(void const *arg) { if (osMessagePut(sensorSetup[SENSOR_0].thrsh_queueId, (uint32_t) sensorSetup[SENSOR_0].last_value, osWaitForever) != osOK) Error_Handler(); }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////









void init_all_task()
{

	int *message;

	message = (int *) osPoolAlloc(thread_poolId);
	*message = TASK_SETUP;

	if (osMessagePut(master_queueId, (uint32_t) message, osWaitForever) != osOK) Error_Handler();
}

void *dataMalloc(int type, int size)
{
	switch (type)
	{
	case INT8:
		return pvPortMalloc(sizeof(int8_t) * size);

	case INT16:
		return pvPortMalloc(sizeof(int16_t) * size);

	case INT32:
		return pvPortMalloc(sizeof(int32_t) * size);

	case FLOAT:
		return pvPortMalloc(sizeof(float) * size);

	case AXISRAW:
		return pvPortMalloc(sizeof(SensorAxesRaw_t) * size);

	case AXIS:
		return pvPortMalloc(sizeof(SensorAxes_t) * size);
	}
}

void dataCopy(int type, int *cnt, SensorsData *src, SensorsData *dst)
{
	(*cnt)--;
	switch (type)
	{
	case INT8:
		memcpy((int8_t*) dst->data + ((*cnt) * src->data_len), (int8_t*) src->data, src->data_len * sizeof(int8_t));
		break;

	case INT16:
		memcpy((int16_t*) dst->data + ((*cnt) * src->data_len), (int16_t*) src->data, src->data_len * sizeof(int16_t));
		break;

	case INT32:
		memcpy((int32_t*) dst->data + ((*cnt) * src->data_len), (int32_t*) src->data, src->data_len * sizeof(int32_t));
		break;

	case FLOAT:
//		for (int packing_cnt = src->data_len; packing_cnt; --packing_cnt) *(((float*) dst->data) + ((*cnt) * src->data_len) + packing_cnt) = *((float*) src->data + packing_cnt);
		memcpy((float*) dst->data + ((*cnt) * src->data_len), (float*) src->data, src->data_len * sizeof(float));
		break;

	case AXIS:
//		for (int packing_cnt = src->data_len; packing_cnt; --packing_cnt) *(((float*) dst->data) + ((*cnt) * src->data_len) + packing_cnt) = *((float*) src->data + packing_cnt);
		memcpy((SensorAxes_t*) dst->data + ((*cnt) * src->data_len), (SensorAxes_t*) src->data, src->data_len * sizeof(SensorAxes_t));
		break;
	}
}



void master_timer_Callback(void const *arg)
{
	int *message;

	message = (int *) osPoolAlloc(thread_poolId);
	*message = PERIOD_CHECK;
	osMessagePut(master_queueId, (uint32_t) message, osWaitForever);
}

void master_timer_Start(void)
{
	exec = 1;
	master_timerId = osTimerCreate(osTimer(master_timer), osTimerPeriodic, &exec);			// Create periodic timer
	if (master_timerId) osTimerStart(master_timerId, (float) timer_period_master);       	// start timer
}

void master_timer_Stop(void) { osTimerStop(master_timerId); }



void ble_timer_Callback(void const *arg) { osSemaphoreRelease(ble_semId); }

void ble_timer_Start(void)
{
	exec = 1;
	ble_timerId = 		osTimerCreate(osTimer(ble_timer), osTimerPeriodic, &exec);			// Create periodic timer
	if (ble_timerId) 	osTimerStart(ble_timerId, (float) timer_period_master);
}

void ble_timer_Stop(void) { osTimerStop(ble_timerId); }

void delayedMessage_Callback(void const *arg)
{
	osEvent evt;
	DelayedSensorsDataMessage *delayedMessage;


	evt = osMessageGet(delayed_queueId, osWaitForever);

	if (evt.status == osEventMessage)
	{
		delayedMessage = (DelayedSensorsDataMessage *) evt.value.p;
		if (osMessagePut(delayedMessage->receiver, (uint32_t) delayedMessage->message, osWaitForever) != osOK) Error_Handler();
		osTimerDelete(delayedMessage->timer_id);
		vPortFree(delayedMessage);
	}
}









static void GetBatteryInfoData(void)
{
//  uint32_t soc;
//  int32_t current= 0;
//	uint32_t voltage;

  uint8_t v_mode;

  BSP_GG_Task(STC3115_handle,&v_mode);		// Update Gas Gouge Status

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(STC3115_handle, &voltage);
  BSP_GG_GetCurrent(STC3115_handle, &current);
  BSP_GG_GetSOC(STC3115_handle, &soc);

  #ifdef DEBUG_USB_NOTIFY_TRAMISSION
  STLBLE_PRINTF("Charge= %ld%% Voltage=%ld mV Current= %ld mA\r\n", soc, voltage, current);
  #endif
}



void delayedOsMessagePut(unsigned int msDelay, int sensor_id, osMessageQId receiver, SensorsData *message)
{
	DelayedSensorsDataMessage *delayedMessage;
	delayedMessage = (DelayedSensorsDataMessage *) pvPortMalloc(sizeof(DelayedSensorsDataMessage));

	delayedMessage->timer_id = osTimerCreate(osTimer(delayedTimer), osTimerOnce, &exec);
	delayedMessage->msDelay = msDelay;
	delayedMessage->sensor_id = sensor_id;
	delayedMessage->receiver = receiver;
	delayedMessage->message = message;

	if(osMessagePut(delayed_queueId, (uint32_t) delayedMessage, osWaitForever) != osOK) Error_Handler();

	osTimerStart(delayedMessage->timer_id, delayedMessage->msDelay);
}



static void InitBlueNRGStack(void)
{
	#ifdef DEBUG_USB_CONNECTION_INFO
	{
		STLBLE_PRINTF("STMicroelectronics %s:\r\n"
			"\tVersion %c.%c.%c\r\n"
			"\tSensorTile"
			"\r\n",
		STLBLE_PACKAGENAME, STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH);

	STLBLE_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n" "\tCompiled %s %s"
	#if defined (__IAR_SYSTEMS_ICC__)
		" (IAR)\r\n"
	#elif defined (__CC_ARM)
		" (KEIL)\r\n"
	#elif defined (__GNUC__)
		" (openstm32)\r\n"
	#endif
			,HAL_GetHalVersion() >>24,
			(HAL_GetHalVersion() >>16)&0xFF,
			(HAL_GetHalVersion() >> 8)&0xFF,
			 HAL_GetHalVersion()      &0xFF,
			__DATE__,__TIME__);
	}
	#endif

	const char 		BoardName[16] = {NAME_STLBLE,0}; // BoardName[8]
	uint16_t 		service_handle, dev_name_char_handle, appearance_char_handle;
	int 			ret;
	uint8_t  		hwVersion;
	uint16_t 		fwVersion;

	#ifdef STATIC_BLE_MAC
	uint8_t tmp_bdaddr[6] = {STATIC_BLE_MAC};
    for (uint8_t i = 0; i < 6; i++) bdaddr[i] = tmp_bdaddr[i];
	#endif


	BNRG_SPI_Init();  															// Initialize the BlueNRG SPI driver
	HCI_Init();																	// Initialize the BlueNRG HCI
	BlueNRG_RST();																// Reset BlueNRG hardware
	getBlueNRGVersion(&hwVersion, &fwVersion);									// Get the BlueNRG HW and FW versions

	if (hwVersion > 0x30) TargetBoardFeatures.bnrg_expansion_board = IDB05A1;	// X-NUCLEO-IDB05A1 expansion board is used
	else TargetBoardFeatures.bnrg_expansion_board = IDB04A1;					// X-NUCLEO-IDB0041 expansion board is used

	BlueNRG_RST();																// Reset BlueNRG again otherwise it will fail.

	#ifndef STATIC_BLE_MAC														// Create a Unique BLE MAC
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((STLBLE_VERSION_MAJOR-48)*10) + (STLBLE_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0;															// For a Legal BLE Random MAC

	#else
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
    if (ret)
    {
		#ifdef DEBUG_USB_CONNECTION_INFO
    	{
    		STLBLE_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
    	}
		#endif
		goto fail;
	}
	#endif

	ret = aci_gatt_init();
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
    	{
    		STLBLE_PRINTF("\r\nGATT_Init failed\r\n");
        }
    	#endif
		goto fail;
	}

	if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1)
	{
		ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}
	else
	{
		ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}

	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nGAP_Init failed\r\n");
        }
    	#endif
		goto fail;
	}

	#ifndef  STATIC_BLE_MAC
	ret = hci_le_set_random_address(bdaddr);
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
		}
		#endif
		goto fail;
	}
	#endif

//	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, 7/*strlen(BoardName)*/, (uint8_t *) BoardName);
	ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(BoardName), (uint8_t *) BoardName); // BLE name "OLOK"
	if (ret)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
		}
		#endif
		while (1);
	}

	ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED, OOB_AUTH_DATA_ABSENT, NULL, 7, 16, USE_FIXED_PIN_FOR_PAIRING, 123456, BONDING);
	if (ret != BLE_STATUS_SUCCESS)
	{
		#ifdef DEBUG_USB_CONNECTION_INFO
		{
			STLBLE_PRINTF("\r\nGAP setting Authentication failed\r\n");
		}
		#endif
		goto fail;
	}

	#ifdef DEBUG_USB_CONNECTION_INFO
	{
		STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
		"\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
		"\t\tBoardName= %s\r\n"
		"\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
		"SensorTile",
		hwVersion,
		fwVersion>>8,
		(fwVersion>>4)&0xF,
		(hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
		BoardName,
		bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
	}
	#endif

	aci_hal_set_tx_power_level(0,7);	// Set BLE output power level, default (0,3), (0,7)

	return;

	fail:
		return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case BNRG_SPI_EXTI_PIN:
			HCI_Isr();
			HCI_ProcessEvent=1;
			break;

		default:

			break;
	}
}



void powerControl(int mode, int frequency)
{

	switch (mode)
	{
		/**
		  * @brief Configure the main internal regulator output voltage.
		  * @param  VoltageScaling: specifies the regulator output voltage to achieve
		  *         a tradeoff between performance and power consumption.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE1 Regulator voltage output range 1 mode,
		  *                                                typical output voltage at 1.2 V,
		  *                                                system frequency up to 80 MHz.
		  *            @arg @ref PWR_REGULATOR_VOLTAGE_SCALE2 Regulator voltage output range 2 mode,
		  *                                                typical output voltage at 1.0 V,
		  *                                                system frequency up to 26 MHz.
		  * @note  When moving from Range 1 to Range 2, the system frequency must be decreased to
		  *        a value below 26 MHz before calling HAL_PWREx_ControlVoltageScaling() API.
		  *        When moving from Range 2 to Range 1, the system frequency can be increased to
		  *        a value up to 80 MHz after calling HAL_PWREx_ControlVoltageScaling() API.
		  * @note  When moving from Range 2 to Range 1, the API waits for VOSF flag to be
		  *        cleared before returning the status. If the flag is not cleared within
		  *        50 microseconds, HAL_TIMEOUT status is reported.
		  * @retval HAL Status
		  */
		case RUN:

			HAL_PWREx_DisableLowPowerRunMode();
			if(frequency < 26)
			{
				HAL_RCC_DeInit();
				SystemClock_Config_adv(frequency);
				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );

				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
//				HAL_PWREx_EnableLowPowerRunMode();
			}
			else
			{
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

				HAL_RCC_DeInit();
				SystemClock_Config_adv_mod(frequency);
				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
			}

		break;



		/**
		  * @brief Enter Low-power Run mode
		  * @note  In Low-power Run mode, all I/O pins keep the same state as in Run mode.
		  * @note  When Regulator is set to PWR_LOWPOWERREGULATOR_ON, the user can optionally configure the
		  *        Flash in power-down monde in setting the RUN_PD bit in FLASH_ACR register.
		  *        Additionally, the clock frequency must be reduced below 2 MHz.
		  *        Setting RUN_PD in FLASH_ACR then appropriately reducing the clock frequency must
		  *        be done before calling HAL_PWREx_EnableLowPowerRunMode() API.
		  * @retval None
		  */
		case LPRUN:
			if(frequency == 1){
				HAL_RCC_DeInit();
				SystemClock_Config_lp();
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
			}
			else{
				HAL_RCC_DeInit();
				SystemClock_Config_lp_2();
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

//				HAL_RCC_DeInit();
//				SystemClock_Config_adv(frequency);
//				ulTimerCountsForOneTick = ( ( SystemCoreClock ) / configTICK_RATE_HZ );
//
//				HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);
				HAL_PWREx_EnableLowPowerRunMode();
			}
		break;



		/**
		  * @brief Enter Sleep or Low-power Sleep mode.
		  * @note  In Sleep/Low-power Sleep mode, all I/O pins keep the same state as in Run mode.
		  * @param Regulator: Specifies the regulator state in Sleep/Low-power Sleep mode.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_MAINREGULATOR_ON Sleep mode (regulator in main mode)
		  *            @arg @ref PWR_LOWPOWERREGULATOR_ON Low-power Sleep mode (regulator in low-power mode)
		  * @note  Low-power Sleep mode is entered from Low-power Run mode. Therefore, if not yet
		  *        in Low-power Run mode before calling HAL_PWR_EnterSLEEPMode() with Regulator set
		  *        to PWR_LOWPOWERREGULATOR_ON, the user can optionally configure the
		  *        Flash in power-down monde in setting the SLEEP_PD bit in FLASH_ACR register.
		  *        Additionally, the clock frequency must be reduced below 2 MHz.
		  *        Setting SLEEP_PD in FLASH_ACR then appropriately reducing the clock frequency must
		  *        be done before calling HAL_PWR_EnterSLEEPMode() API.
		  * @note  When exiting Low-power Sleep mode, the MCU is in Low-power Run mode. To move in
		  *        Run mode, the user must resort to HAL_PWREx_DisableLowPowerRunMode() API.
		  * @param SLEEPEntry: Specifies if Sleep mode is entered with WFI or WFE instruction.
		  *           This parameter can be one of the following values:
		  *            @arg @ref PWR_SLEEPENTRY_WFI enter Sleep or Low-power Sleep mode with WFI instruction
		  *            @arg @ref PWR_SLEEPENTRY_WFE enter Sleep or Low-power Sleep mode with WFE instruction
		  * @note  When WFI entry is used, tick interrupt have to be disabled if not desired as
		  *        the interrupt wake up source.
		  * @retval None
		  */
		case SLEEP_WFI:
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		break;

		case SLEEP_WFE:
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		break;

		case LPSLEEP_WFI:
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		break;

		case LPSLEEP_WFE:
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 0 mode.
		  * @note  In Stop 0 mode, main and low voltage regulators are ON.
		  * @note  In Stop 0 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped; the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with the wakeup capability
		  *        (I2Cx, USARTx and LPUART) can switch on the HSI to receive a frame, and switch off the HSI
		  *        after receiving the frame if it is not a wakeup frame. In this case, the HSI clock is propagated
		  *        only to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  * @note  When exiting Stop 0 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @note  By keeping the internal regulator ON during Stop 0 mode, the consumption
		  *         is higher although the startup time is reduced.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */
		case STOP0_WFI:
//			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP0_WFE:
//			HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE);
			HAL_PWREx_EnterSTOP0Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 1 mode.
		  * @note  In Stop 1 mode, only low power voltage regulator is ON.
		  * @note  In Stop 1 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped; the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with the wakeup capability
		  *        (I2Cx, USARTx and LPUART) can switch on the HSI to receive a frame, and switch off the HSI
		  *        after receiving the frame if it is not a wakeup frame. In this case, the HSI clock is propagated
		  *        only to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  * @note  When exiting Stop 1 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @note  Due to low power mode, an additional startup delay is incurred when waking up from Stop 1 mode.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */
		case STOP1_WFI:
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP1_WFE:
//			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
			HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Stop 2 mode.
		  * @note  In Stop 2 mode, only low power voltage regulator is ON.
		  * @note  In Stop 2 mode, all I/O pins keep the same state as in Run mode.
		  * @note  All clocks in the VCORE domain are stopped, the PLL, the MSI,
		  *        the HSI and the HSE oscillators are disabled. Some peripherals with wakeup capability
		  *        (LCD, LPTIM1, I2C3 and LPUART) can switch on the HSI to receive a frame, and switch off the HSI after
		  *        receiving the frame if it is not a wakeup frame. In this case the HSI clock is propagated only
		  *        to the peripheral requesting it.
		  *        SRAM1, SRAM2 and register contents are preserved.
		  *        The BOR is available.
		  *        The voltage regulator is set in low-power mode but LPR bit must be cleared to enter stop 2 mode.
		  *        Otherwise, Stop 1 mode is entered.
		  * @note  When exiting Stop 2 mode by issuing an interrupt or a wakeup event,
		  *         the HSI RC oscillator is selected as system clock if STOPWUCK bit in RCC_CFGR register
		  *         is set; the MSI oscillator is selected if STOPWUCK is cleared.
		  * @param STOPEntry  specifies if Stop mode in entered with WFI or WFE instruction.
		  *          This parameter can be one of the following values:
		  *            @arg @ref PWR_STOPENTRY_WFI  Enter Stop mode with WFI instruction
		  *            @arg @ref PWR_STOPENTRY_WFE  Enter Stop mode with WFE instruction
		  * @retval None
		  */

		case STOP2_WFI:
			HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		break;

		case STOP2_WFE:
			HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);
		break;



		/**
		  * @brief Enter Standby mode.
		  * @note  In Standby mode, the PLL, the HSI, the MSI and the HSE oscillators are switched
		  *        off. The voltage regulator is disabled, except when SRAM2 content is preserved
		  *        in which case the regulator is in low-power mode.
		  *        SRAM1 and register contents are lost except for registers in the Backup domain and
		  *        Standby circuitry. SRAM2 content can be preserved if the bit RRS is set in PWR_CR3 register.
		  *        To enable this feature, the user can resort to HAL_PWREx_EnableSRAM2ContentRetention() API
		  *        to set RRS bit.
		  *        The BOR is available.
		  * @note  The I/Os can be configured either with a pull-up or pull-down or can be kept in analog state.
		  *        HAL_PWREx_EnableGPIOPullUp() and HAL_PWREx_EnableGPIOPullDown() respectively enable Pull Up and
		  *        Pull Down state, HAL_PWREx_DisableGPIOPullUp() and HAL_PWREx_DisableGPIOPullDown() disable the
		  *        same.
		  *        These states are effective in Standby mode only if APC bit is set through
		  *        HAL_PWREx_EnablePullUpPullDownConfig() API.
		  * @retval None
		  */
		case STANDBY:
			HAL_PWR_EnterSTANDBYMode();
		break;



		/**
		  * @brief Enter Shutdown mode.
		  * @note  In Shutdown mode, the PLL, the HSI, the MSI, the LSI and the HSE oscillators are switched
		  *        off. The voltage regulator is disabled and Vcore domain is powered off.
		  *        SRAM1, SRAM2 and registers contents are lost except for registers in the Backup domain.
		  *        The BOR is not available.
		  * @note  The I/Os can be configured either with a pull-up or pull-down or can be kept in analog state.
		  * @retval None
		  */
		case SHUTDOWN:
			HAL_PWREx_EnterSHUTDOWNMode();
		break;



		default:
			while(1);
		break;
	}
}

/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeGyro( void )
{
	  if (BSP_GYRO_Init( LSM6DSM_G_0, &LSM6DSM_G_0_handle ) != COMPONENT_OK)
	  {
	    while(1);
	  }

}


/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableGyro( void )
 {
   BSP_GYRO_Sensor_Enable( LSM6DSM_G_0_handle );
 }

///**
// * @brief Get the gyroscope sensor axes
// * @param handle the device handle
// * @param angular_velocity pointer where the values of the axes are written [mdps]
// * @retval COMPONENT_OK in case of success
// * @retval COMPONENT_ERROR in case of failure
// */
//DrvStatusTypeDef BSP_GYRO_Get_Axes( void *handle, SensorAxes_t *angular_velocity )
//{
//
//  DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
//  GYRO_Drv_t *driver = NULL;
//
//  if(ctx == NULL)
//  {
//    return COMPONENT_ERROR;
//  }
//
//  driver = ( GYRO_Drv_t * )ctx->pVTable;
//
//  if ( angular_velocity == NULL )
//  {
//    return COMPONENT_ERROR;
//  }
//  if ( driver->Get_Axes == NULL )
//  {
//    return COMPONENT_ERROR;
//  }
//  if ( driver->Get_Axes( ctx, angular_velocity ) == COMPONENT_ERROR )
//  {
//    return COMPONENT_ERROR;
//  }
//
//  return COMPONENT_OK;
//}

/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeAcc( void )
{
	if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
	{
		  while(1);
	}

}


/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void enableAcc( void )
 {
   BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
 }


 /**
 * @brief  Set ODR all sensors
 * @param  None
 * @retval None
 */
 void setOdrAcc( void )
 {
   BSP_ACCELERO_Set_ODR_Value( LSM303AGR_X_0_handle, ACCELERO_ODR);
 }


 /**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
 void disableAcc( void )
 {
   BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
 }

 DrvStatusTypeDef BSP_ACCELERO_GetData( void *handle, SensorAxes_t *acceleration )
 {

   DrvContextTypeDef *ctx = (DrvContextTypeDef *)handle;
   ACCELERO_Drv_t *driver = NULL;

   if(ctx == NULL)
   {
     return COMPONENT_ERROR;
   }

   driver = ( ACCELERO_Drv_t * )ctx->pVTable;

   if(acceleration == NULL)
   {
     return COMPONENT_ERROR;
   }

   if ( driver->Get_Axes == NULL )
   {
     return COMPONENT_ERROR;
   }

   if ( driver->Get_Axes( ctx, acceleration ) == COMPONENT_ERROR )
   {
     return COMPONENT_ERROR;
   }

   return COMPONENT_OK;
 }



void Error_Handler(void) { while (1) {} }



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */



void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif
