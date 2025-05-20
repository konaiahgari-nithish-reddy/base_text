/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "main.h"
#include "PeripheralsInit.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"

#include <GenericTypeDefs.h>
#include "config.h"
#define	DEFINE_GLOBALS
// NOTE: all header files that define global variables MUST be included here, even if the globals are not otherwise used in this file
#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "MenuFSM.h"
//#include "SST25VF016.h"		// SPI Flash function definitions
#include "MotionProfile.h"
#include "EventFlags.h"			// event flag definitions and globals
#include "Debounce.h"
#include "DS3232.h"				// Real Time Clock
#ifdef USE_MMA8452Q_INCLINOMETER
#include "mma845x.h"              // MMA845xQ definitions
#include "Inclinometer.h"
#endif
#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"		// Sun Position FSM - operating modes
#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"
#include "ButtonProcessingFSM.h" // Button and user input processing
#include "RxMessage.h"
#include "CoordTranslate.h"
#include "SerialDisplay.h"
#include "RTCC.h"
#include "Drv8245.h"
#include "LIS2HH12_Accelerometer.h"
#include "dwt_delay.h"
#include "ESP32FSM.h"
#undef	DEFINE_GLOBALS
#include "wifi_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
char ms5_cnt = 0,ms25_cnt=0,ms100_cnt=0;
BOOL ms5_fl=0,ms25_fl=0,ms100_fl=0;
volatile uint32_t timer_count = 0;
//volatile uint8_t timer_flag = 0;


//volatile uint8_t timer_flag = 0;
extern volatile uint32_t wifi_ctr;
extern volatile uint8_t wifi_flag;

extern volatile uint8_t rx_buff[RX_BUFF_SIZE];
extern volatile uint16_t rx_buff_ctr;
extern volatile uint8_t json_packet_flag;
extern volatile uint8_t json_validator;
extern volatile uint16_t json_packet_size;
extern uint8_t setting_flag;


uint32_t WwdgStatus = 0;
#ifdef INTERNAL_IMU
uint8_t imu_flag = 1;
#else
uint8_t imu_flag = 0;
#endif

WWDG_HandleTypeDef hwwdg;
static void WWDG_Init(void);
static uint32_t TimeoutCalculation(uint32_t timevalue);
IWDG_HandleTypeDef hiwdg;
static void MX_IWDG_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* Definitions for defaultTask */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// These are the counting periods for the medium (25mS) and slow (100mS) events, based on a 5mS tick

#define _100MS_EVENT_PERIOD			(unsigned int)4			// 100ms period  @ 25mS
#define	_1S_EVENT_PERIOD			(unsigned int)40		// 1 Sec period @ 25mS

// this is based on the 100mS tick
#define	_15S_EVENT_PERIOD			(unsigned int)150		// 15 Sec period @ 100mS

enum tagMainErrors
{
	MAIN_ERROR_NONE = MAIN_ERROR_BASE,
	MAIN_ERROR_UNEXPECTED_TICK,				// 1 unexpected timer tick event
	MAIN_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	MAIN_ERROR_INVALID_STATE,				// 3 not a valid state
	MAIN_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	MAIN_ERROR_UNKNOWN_COMMAND,				// 5 not a valid command
	MAIN_ERROR_25MS_TICK_OVERRUN,			// 6 unprocessed event flag
	MAIN_ERROR_100MS_TICK_OVERRUN,			// 7 unprocessed event flag
	MAIN_ERROR_1S_TICK_OVERRUN,				// 8 unprocessed event flag
	MAIN_ERROR_25MS_MOTION_TICK_OVERRUN,	// 9 unprocessed event flag

	MAIN_ERROR_UNPROCESSED_EVENT = MAIN_ERROR_BASE + 0x0F
};


// 5mS Tick Time distribution FSM States
enum tag5mSTickStates
{
	ST_5mS_TICK1,
	ST_5mS_TICK2,
	ST_5mS_TICK3,
	ST_5mS_TICK4,
	ST_5mS_TICK5
};

enum tag5mSTickStates e5mSTickState = ST_5mS_TICK1;

enum tag25mSTickStates
{
	ST_None,
	ST_25mS_TICK1,
	ST_25mS_TICK2,
	ST_25mS_TICK3,
	ST_25mS_TICK4,
	ST_25mS_TICK5
};

enum tag25mSTickStates e25mSTickState = ST_None;

enum tag100mSTickStates
{
	None,
	ST_100mS_TICK1,
	ST_100mS_TICK2,
	ST_100mS_TICK3,
	ST_100mS_TICK4
};

enum tag100mSTickStates e100mSTickState = None;

// This is a software counter used to time slower system events, such as button polling, etc.
unsigned int fg100mSTimerCount1 = _100MS_EVENT_PERIOD;
unsigned int fg100mSTimerCount2 = _100MS_EVENT_PERIOD;
unsigned int fg100mSTimerCount3 = _100MS_EVENT_PERIOD;
unsigned int fg100mSTimerCount4 = _100MS_EVENT_PERIOD;

BYTE fgbI2CErrorCtr = 0;	// Track I2C errors when reading Inclinometer
BYTE fgbI2CHrdStlCtr = 0;	// Track I2C errors when reading Inclinometer

PRIVATE_INIT float  fgdefferenceAngle = 0.0;
#define MIN_FLUCT_ANGLE     0.5
#define MIN_MANUAL_ANGLE     2

// these are just dummy structures passed to PanelPositionFSM() for initialization
static SmartTrakOrientation fgOrientation;

BOOL gIsMma845Enabled = TRUE;

void MenuFSM(void);
static void MX_USB_OTG_FS_PCD_Init(void);
//uint32_t DWT_Delay_Init(void);

uint8_t TxMessageBuffer[] = "Device ID: ";
#define VERSION		"2.17\r\n"

volatile uint32_t debug_ctr = 0;
FlagStatus debug_flag = RESET;
FlagStatus debug_cmd = RESET;
extern SmartTrakOrientation acquisitionAngle;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	/* Reset the RTC peripheral and the RTC clock source selection */
//	HAL_PWR_EnableBkUpAccess();
//	__HAL_RCC_BACKUPRESET_FORCE();
//	__HAL_RCC_BACKUPRESET_RELEASE();

  /* Clear reset flags in any case */
    __HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  MX_MEMORYMAP_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
//  MX_USB_OTG_FS_PCD_Init();
  MX_ICACHE_Init();
  MX_TIM4_Init();
  MX_RTC_Init();
#ifdef	WD_ENABLE
  MX_IWDG_Init();
#endif
  /* USER CODE BEGIN 2 */
  /* calculate delay to enter window. Add 1ms to secure round number to upper number  */
//    delay = TimeoutCalculation((hwwdg.Init.Counter-hwwdg.Init.Window) + 1) + 1;

  /* Init Device Library */
#ifdef	USB_LOG_ENABLE
    USBD_Init(&hUsbDeviceFS, &VCP_Desc, 0);

    /* Add Supported Class */
    USBD_RegisterClass(&hUsbDeviceFS, USBD_CDC_CLASS);

    /* Add CDC Interface Class */
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops);

    /* Start Device Process */
    USBD_Start(&hUsbDeviceFS);

    char UserTxBuffer[10] = {0};
    uint32_t uniqueid = *((uint32_t*)0x0BFA0700);
	snprintf(UserTxBuffer, 10, "ST%X", (unsigned int)uniqueid);

    if (hUsbDeviceFS.pClassData != NULL) {
        Transmit(TxMessageBuffer, sizeof(TxMessageBuffer));
        Transmit(UserTxBuffer, sizeof(UserTxBuffer));
	}

    HAL_Delay(1000);
    uart_send("Device ID: ");
    uart_send(UserTxBuffer);
	HAL_Delay(1000);
#endif
#ifdef	WD_ENABLE
    /* Refresh IWDG: reload counter */
     if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
     {
       /* Refresh Error */
       Error_Handler();
     }
#endif

    // Resetting ESP32 Chip
    HAL_GPIO_WritePin(Wifi_Enable_GPIO_Port, Wifi_Enable_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(Wifi_Enable_GPIO_Port, Wifi_Enable_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);
//  BYTE bTemp;
//  	BOOL bRetVal = TRUE;


//  bRetVal = MMA845x_Standby();
//  	if (bRetVal IS FALSE)
//  		return bRetVal;

//  //	IIC_RegRead(SlaveAddressIIC, WHO_AM_I_REG);
//  	bRetVal = ReadInclinometerRegister(WHO_AM_I_REG, &bTemp);
//
//  	if ((bTemp IS MMA8452Q_ID) && (bRetVal IS TRUE))
//  		bRetVal = TRUE;
//  	else
//  		return (FALSE);

	InitializeParameretes();
  /* USER CODE END 2 */

//	 uint8_t  Buf[1000];
	/*while(1)
	{
		if (WIFI_Uart_Recv(Buf, sizeof(Buf)))
		{
			uart_send(Buf);//uart_send(Buf);
			HAL_Delay(100);
			WIFI_Uart_Send("Hello ESP32");
		}
		WIFI_Uart_Send("Hello ESP32");
		HAL_Delay(1000);
	}
	//  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
	//  MX_FREERTOS_Init();

	/* Start scheduler */
	//  osKernelStart();
	BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT);
	ms5_cnt=1;
	ms5_fl=1;

	char debug_buff[100];
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		while(ms5_fl==1)
		{
#ifdef	WD_ENABLE
		    HAL_IWDG_Refresh(&hiwdg);
#endif
			uint8_t ch = 0;

			if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_3) == GPIO_PIN_SET) {
				  HAL_GPIO_WritePin(Wifi_Enable_GPIO_Port, Wifi_Enable_Pin, GPIO_PIN_RESET);
				  HAL_Delay(1000);
				  HAL_GPIO_WritePin(Wifi_Enable_GPIO_Port, Wifi_Enable_Pin, GPIO_PIN_SET);
				  HAL_Delay(1000);
			}

			// Data Acquisition for Transmission
			if (wifi_flag == SET) {
				// Read Transmission Parameter
				read_monitor_params();
				create_mon_mqtt_packet();
				// Transmit via UART in blocking mode

				// Read packet reception ACK from ESP32
				wifi_ctr = 0;
				wifi_flag = RESET;
			}

			if(HAL_UART_Receive(&hlpuart1, &ch, 1, 1) == HAL_OK)
			{
				if (ch == '{') {
					json_validator++;
					if (json_validator<2) {
						memset(rx_buff, 0, RX_BUFF_SIZE);
					}
				}
				if (ch == '}') {
					json_validator--;
					rx_buff[rx_buff_ctr++] = ch;
					if (json_validator == 0) {
						json_packet_flag = 1;
					}
				}
				if (json_validator > 0) {
					rx_buff[rx_buff_ctr++] = ch;
				}

			}
			if (json_packet_flag == 1) {
				json_packet_flag = 0;
				json_packet_size = 0;
				rx_buff_ctr = 0;
				json_validator = 0;
				parse_stm32_packet(rx_buff, strlen(rx_buff));
				memset(rx_buff, 0, RX_BUFF_SIZE);
			}

			if (setting_flag != 0) {
				read_setting_params();
				create_read_setting_mqtt();
				setting_flag = 0;
			}

			if (debug_flag == SET && debug_cmd == SET) {
				debug_flag = RESET;
				memset(debug_buff, 0, sizeof(debug_buff));
				snprintf(debug_buff, 85, "Sun_EL: %f\tSun_Az: %f\tTarget: %f \tTracker: %f \n",ptrRTCC_RAM_AppParameters->fSPACalculation_EL, ptrRTCC_RAM_AppParameters->fSPACalculation_AZ, acquisitionAngle.fModuleTiltAngleDegrees, pgAngleAverage.fX_Angle);
				Transmit(debug_buff, sizeof(debug_buff));
			}
			if (imu_flag == 1) {
				MainMenu('i');
				imu_flag = 0;
			}
#ifdef USE_AZIMUTH

			if (IS_BITSET(efMotionSensorEvents[MOTOR_AZIMUTH], EF_MOTION_SENSOR_TICK))
			{
				MotionSensor_Tick(MOTOR_AZIMUTH);			// processes current speed for PWM adjustments

				//continue;				// we have processed something, so restart at the top of the loop
			}
#endif
#ifdef USE_ELEVATION
			if (IS_BITSET(efMotionSensorEvents[MOTOR_ELEVATION], EF_MOTION_SENSOR_TICK))
			{
				MotionSensor_Tick(MOTOR_ELEVATION);			// processes current speed for PWM adjustments

				//continue;				// we have processed something, so restart at the top of the loop
			}
#endif

			if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
			{
				//									// clear event flag (5mS event overrun is handled in the 5mS timer interrupt)
				BITCLEAR(efTimerEvents, EF_TIMER_5MS_TICK_INT);

				//-------------------------------
				//	Switch Debounce on all 5mS Ticks
				//-------------------------------
				// handle 5mS events


#ifdef USE_PCA9554_IO					// enabled in config.h ONLY if PCA9554 for Switch Input is present
#ifdef USE_DEBOUNCE_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

				Input_Debounce_FSM(FALSE);			// input switch debounce handling, called unconditionally every 5mS

#ifdef USE_DEBOUNCE_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif
#endif
				switch (e5mSTickState)
				{
				//-------------------------------
				//			5mS Tick 1
				//-------------------------------
				case ST_5mS_TICK1:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1))
					{
						//					RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
					}
//															uart_send("\r\n5ms tick 1\r\n");
					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1);
					e25mSTickState = ST_25mS_TICK1;
					e5mSTickState = ST_5mS_TICK2;			// bump state for next tick
					break;

					//-------------------------------
					//			5mS Tick 2
					//-------------------------------
				case ST_5mS_TICK2:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK2))
					{
						//					RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
					}
					//											uart_send("\r\n5ms tick 2\r\n");
					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK2);
					e25mSTickState = ST_25mS_TICK2;
					e5mSTickState = ST_5mS_TICK3;			// bump state for next tick
					break;

					//-------------------------------
					//			5mS Tick 3
					//-------------------------------
				case ST_5mS_TICK3:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3))
					{
						//					RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
					}
					//										uart_send("\r\n5ms tick 3\r\n");
					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3);
					e25mSTickState = ST_25mS_TICK3;
					e5mSTickState = ST_5mS_TICK4;			// bump state for next tick
					break;

					//-------------------------------
					//			5mS Tick 4
					//-------------------------------
				case ST_5mS_TICK4:
					//-------------------------------
					//	25mS Azimuth Motion Timer
					//-------------------------------
					// ==> this delay may be anything UP TO 25mS; the start of the delay is asynchronous in MotionFSM(), but the end is always on Tick4
					if (IS_BITSET(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER))
					{
						// Azimuth motion timer is enabled
						// check for 25mS Motion Timer event overrun
						if (IS_BITSET(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK))
						{
							//						RuntimeError(MAIN_ERROR_25MS_MOTION_TICK_OVERRUN);
						}

						// set event flag; processed by MotionFSM()
						BITSET(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK);
					}
					else
					{
						// Azimuth Motion Timer is NOT running
						// make sure event is reset for next use
						BITCLEAR(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK);
					}
					//										uart_send("\r\n5ms tick 4\r\n");
					//-------------------------------
					//	25mS Elevation Motion Timer
					//-------------------------------
					if (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER))
					{
						// Elevation motion timer is enabled
						// check for 25mS Motion Timer event overrun
						if (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK))
						{
							//						RuntimeError(MAIN_ERROR_25MS_MOTION_TICK_OVERRUN);
						}

						// set event flag; processed by MotionFSM()
						BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK);
					}
					else
					{
						// Elevation Motion Timer is NOT running
						// make sure event is reset for next use
						BITCLEAR(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK);
					}

					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK4))
					{
						//					RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
					}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK4);
					e25mSTickState = ST_25mS_TICK4;
					e5mSTickState = ST_5mS_TICK5;			// bump state for next tick
					break;

					//-------------------------------
					//			5mS Tick 5
					//-------------------------------
				case ST_5mS_TICK5:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5))
					{
						//					RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
					}
					//									uart_send("\r\n5ms tick 5\r\n");
					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5);
					e25mSTickState = ST_25mS_TICK5;
					e5mSTickState = ST_5mS_TICK1;			// bump state for next tick
					break;
				}

				//-----------------------------------------------------------------
				// Check for non-timer based events to be processed every 5mS
				//-----------------------------------------------------------------

				// these events should be executed whenever there are non-zero event flags - but not SO often that they completely tie up the system.
				// a sticky (unprocessed) event here will completely kill the system..

#ifdef USE_FEEDBACK_SIMULATOR
				// executes on EVERY 5mS tick
				//-------------------------------
				//		Feedback Simulator
				//-------------------------------
				if (pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH] != 0)
				{
					--pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH];

					// check for end of count
					if (pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH] == 0)
					{
						// counter has expired, so reload the simulated Motion Sensor tick counter
						MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);

						// create a Motion Sensor Feedback pulse
						SetFeedbackSimulator(MOTOR_AZIMUTH);					// feedback pin HIGH
						// Motion Sensor interrupt will occur HERE
						ClearFeedbackSimulator(MOTOR_AZIMUTH);					// feedback pin LOW
					}
				}

				if (pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION] != 0)
				{
					--pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION];

					// check for end of count
					if (pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION] == 0)
					{
						// counter has expired, so reload the simulated Motion Sensor tick counter
						MS_SetSimulatorTickCtr(MOTOR_ELEVATION);

						// create a Motion Sensor Feedback pulse
						SetFeedbackSimulator(MOTOR_ELEVATION);					// feedback pin HIGH
						// Motion Sensor interrupt will occur HERE
						ClearFeedbackSimulator(MOTOR_ELEVATION);				// feedback pin LOW
					}
				}

#endif	//  USE_FEEDBACK_SIMULATOR

				//-------------------------------
				//	MotionPhaseFSM 5mS Call
				//-------------------------------
				// should these be split across 5mS ticks?
#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

				if ( ((efMotionResultEvents[MOTOR_AZIMUTH] != NO_EVENTFLAGS) /* || ((efSwitchEvents & EF_SWITCH_UP_AZ_EVENTS_MASK) != (WORD)0) */)
						&& (pgeMotionPhase[MOTOR_AZIMUTH] != PHASE_OPEN_LOOP) )
				{
					MotionPhaseFSM(MOTOR_AZIMUTH);				// event(s) to be processed, so run the Move Command FSM without waiting for the timer
				}

				if ( ((efMotionResultEvents[MOTOR_ELEVATION] != NO_EVENTFLAGS) /* || ((efSwitchEvents & EF_SWITCH_UP_EL_EVENTS_MASK) != (WORD)0) */)
						&& (pgeMotionPhase[MOTOR_ELEVATION] != PHASE_OPEN_LOOP) )
				{
					MotionPhaseFSM(MOTOR_ELEVATION);			// event(s) to be processed, so run the Move Command FSM without waiting for the timer
				}
#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

				//-------------------------------
				//		MotionFSM 5mS call
				//-------------------------------
				// should these be split across 5mS ticks?
				// run the MotionFSM second, because it has some slightly sticky events that would prevent the MotionPhaseFSM from running
#ifdef USE_MOTION_FSM_5MS_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif
				if ( ((efMotionEvents[MOTOR_AZIMUTH] != NO_EVENTFLAGS) /*|| (IsMotionComplete(MOTOR_AZIMUTH) == 0)*/ /*|| (IS_BITSET(efTimerEvents, EF_TIMER_MOTION_25MS_TICK))*/)		// <== kludge?
						&& (pgeMotionPhase[MOTOR_AZIMUTH] != PHASE_OPEN_LOOP) )

				{
					MotionFSM(MOTOR_AZIMUTH);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

				if ( ((efMotionEvents[MOTOR_ELEVATION] != NO_EVENTFLAGS)/* || (IsMotionComplete(MOTOR_ELEVATION) == 0)*/ /* || (IS_BITSET(efTimerEvents, EF_TIMER_MOTION_25MS_TICK))*/)		// <== kludge?
						&& (pgeMotionPhase[MOTOR_ELEVATION] != PHASE_OPEN_LOOP) )
				{
					MotionFSM(MOTOR_ELEVATION);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

			}		// end if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
#ifdef USE_MOTION_FSM_5MS_TRIGGER
			Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

			//-----------------------------------------------------------------
			//		End of 5ms Events
			//-----------------------------------------------------------------

			//-----------------------------------------------------------------
			//		Handle 25mS Tick events
			//-----------------------------------------------------------------
			// these events are tied to 25mS ticks, and the use of multiple 25mS tick flags forces them to be distributed in time

			//-------------------------------
			//			25mS Tick 1
			//-------------------------------
			// tick:		(efTimerEvents, EF_TIMER_25MS_TICK1)
			// duration:
			// counter:
			// output:
			//				switch (e25mSTickState)
			//				{
			//					case ST_None:
			////						uart_send("\r\n25ms tick none\r\n");
			//						break;
			//					case ST_25mS_TICK1:
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1))
			{
				// clear calling event
				BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK1);
#ifdef USE_MENU_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif
				//						uart_send("\r\n25ms tick 1\r\n");
#ifdef UART_DUAL_TEST
				MenuFSM();
#endif
#ifdef RXMSG
				//						RxMessageFSM(SERIAL_REMOTE_UART);
#endif
#ifdef BOTH_UARTS
				eSerialOutputMode = SER_MODE_MENU;
				MenuFSM();
				// MenuFSM(SERIAL_REMOTE_UART);
				//				eSerialOutputMode = SER_MODE_MENU;
				//   eSerialOutputMode = SER_MODE_REMOTE;
				//				RxMessageFSM();
#endif
#ifdef USE_MENU_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif
				//						break;
				continue;							// we have processed something, so restart at the top of the loop
			}
			//					case ST_25mS_TICK2:
			//-------------------------------
			//			25mS Tick 2
			//-------------------------------
			// enabled by:	none
			// tick:		(efTimerEvents, EF_TIMER_25MS_TICK2)
			// duration:	1 tick
			// counter:		none
			// output:		call MotionFSM

			if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK2))
			{

				// bump 100mS tick counter
				fg100mSTimerCount1--;
				//							uart_send("\r\n25ms tick 2\r\n");
				// check for timer complete - or (ACK!) rolled under
				if ((fg100mSTimerCount1 == (unsigned int)0) || (fg100mSTimerCount1 > _100MS_EVENT_PERIOD))
				{
					// no need to check for overrun on 100mS events

					// set event flag
					BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK1);
					e100mSTickState = ST_100mS_TICK1;
					// reload timer
					fg100mSTimerCount1 = _100MS_EVENT_PERIOD;
				}
				// if the MotionPhase is PHASE_OPEN_LOOP, we are running OpenLoop menus selections, so there is no need to run MotionFSM()
				// if the (efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK) bit is set, we are ALREADY calling the MotionFSM() on a timed basis on 25mS Tick4, so we can skip this call...  16 Apr 13 <sek>

#ifdef USE_MOTION_FSM_25MS_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

				if ((pgeMotionPhase[MOTOR_AZIMUTH] != PHASE_OPEN_LOOP) && (IS_BITCLEAR(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK)))
				{
					// run Motion Control FSM on a timed basis
					MotionFSM(MOTOR_AZIMUTH);
				}

#ifdef USE_ELEVATION
				// if the MotionPhase is PHASE_OPEN_LOOP, we are running OpenLoop menus selections, so there is no need to run MotionFSM()
				// if the (efMotionTimerEvents[MOTOR_AZIMUTH], MOTOR_ELEVATION) bit is set, we are ALREADY calling the MotionFSM() on a timed basis, so we can skip this call...  16 Apr 13 <sek>
				if ((pgeMotionPhase[MOTOR_ELEVATION] != PHASE_OPEN_LOOP) && (IS_BITCLEAR(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK)))
				{
					// run Motion Control FSM on a timed basis
					MotionFSM(MOTOR_ELEVATION);
				}
#endif	// USE_ELEVATION

#ifdef USE_MOTION_FSM_25MS_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

				// clear calling event
				BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK2);

				//							break;
				continue;				// we have processed something, so restart at the top of the loop
			}

			//					case ST_25mS_TICK3:
			//-------------------------------
			//			25mS Tick 3
			//-------------------------------
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3))
			{
				// run Menu FSM on a timed basis
#ifndef UART_TEST
				MenuFSM();
#endif
				//						uart_send("\r\n25ms tick 3\r\n");
				// bump 100mS tick counter
				fg100mSTimerCount2--;

				// check for timer complete - or (ACK!) rolled under
				if ((fg100mSTimerCount2 == (unsigned int)0) || (fg100mSTimerCount2 > _100MS_EVENT_PERIOD))
				{
					// no need to check for overrun on 100mS events

					// set event flag
					BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK2);
					e100mSTickState = ST_100mS_TICK2;
					// reload timer
					fg100mSTimerCount2 = _100MS_EVENT_PERIOD;
				}
				// clear calling event
				BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK3);
				//							break;
				continue;								// we have processed something, so restart at the top of the loop
				// this prevents processing EF_SCHED_25MS_TICK3 and EF_SCHED_100MS_TICK1 on the same 5mS tick
			}
			//					case ST_25mS_TICK4:
			//-------------------------------
			//			25mS Tick 4
			//		Motion Timer Processing
			//-------------------------------
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK4))
			{
				// bump 100mS tick counter
				fg100mSTimerCount3--;
				//						uart_send("\r\n25ms tick 4\r\n");
				// check for timer complete - or (ACK!) rolled under
				if ((fg100mSTimerCount3 == (unsigned int)0) ||  (fg100mSTimerCount3 > _100MS_EVENT_PERIOD))
				{
					// no need to check for overrun on 100mS events

					// set event flag
					BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK3);
					e100mSTickState = ST_100mS_TICK3;
					// reload timer
					fg100mSTimerCount3 = _100MS_EVENT_PERIOD;
				}

				//-------------------------------
				//	MotionFSM Motion Timer Processing
				//-------------------------------
				// if the MotionPhase is PHASE_OPEN_LOOP, we are running OpenLoop menus selections, so there is no need to run MotionFSM()
				// call MotionFSM() ONLY if the Motion Timer is running
#ifdef USE_MOTION_FSM_TIMER_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

				if ( (IS_BITSET(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK)) && (pgeMotionPhase[MOTOR_AZIMUTH] != PHASE_OPEN_LOOP) )
				{
					MotionFSM(MOTOR_AZIMUTH);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

#ifdef USE_ELEVATION
				// perhaps this should be moved to the next 25mS tick?
				if ( (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK)) && (pgeMotionPhase[MOTOR_ELEVATION] != PHASE_OPEN_LOOP) )
				{
					MotionFSM(MOTOR_ELEVATION);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}
#endif	// USE_ELEVATION

#ifdef USE_MOTION_FSM_TIMER_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

				// clear calling event
				BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK4);
				//							break;
				continue;								// we have processed something, so restart at the top of the loop
				// this prevents processing EF_SCHED_25MS_TICK4 and EF_SCHED_100MS_TICK2 on the same 5mS tick
			}
			//					case ST_25mS_TICK5:
			//-------------------------------
			//			25mS Tick 5
			//-------------------------------
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5))
			{
				// bump 100mS tick counter
				fg100mSTimerCount4--;
				//						uart_send("\r\n25ms tick 5\r\n");
				// check for timer complete - or (ACK!) rolled under
				if ((fg100mSTimerCount4 == (unsigned int)0) ||  (fg100mSTimerCount4 > _100MS_EVENT_PERIOD))
				{
					// no need to check for overrun on 100mS events

					// set event flag
					BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK4);
					e100mSTickState = ST_100mS_TICK4;
					// reload timer
					fg100mSTimerCount4 = _100MS_EVENT_PERIOD;
				}
				// clear calling event
				BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK5);
				//						break;
				continue;									// we have processed something, so restart at the top of the loop
				// this prevents processing EF_SCHED_25MS_TICK5 and EF_SCHED_100MS_TICK3 on the same 5mS tick
			}
			//-----------------------------------------------------------------
			//		Handle 100mS Tick events
			//-----------------------------------------------------------------
			//				switch (e100mSTickState)
			//				{
			//					case None:
			//						break;
			//					case ST_100mS_TICK1:

			if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK1))
			{
#ifdef USE_MOVE_SEQ_TICK_TRIGGER
				Trigger1Level(1);						// trigger to allow viewing this event on a scope
#endif
				//						uart_send("\r\n100ms tick 1\r\n");
				MoveSequenceTick();							// MoveSequenceFSM Timers()
#ifdef USE_MOVE_SEQ_TICK_TRIGGER
				Trigger1Level(0);						// trigger to allow viewing this event on a scope
#endif


#ifdef USE_BUTTON_PROC_FSM_TRIGGER
				//							Trigger1Level(1);						// trigger to allow viewing this event on a scope
#endif

#ifdef SWITCH_INIT
				Switch_processing();
#endif

				//						ButtonProcessingFSM();

#ifdef USE_BUTTON_PROC_FSM_TRIGGER
				//							Trigger1Level(0);						// trigger to allow viewing this event on a scope
#endif

#ifdef USE_HARD_STALL_RESET_TIMER
				// handle Hard Stall Reset Delay Timer
				if (IS_BITSET(efTimerEvents, EF_TIMER_HARD_STALL))
				{
					// Hard Stall Reset Delay timer is enabled

					// bump Hard Stall Reset Delay timer
					fgHardStallResetDelayTimerCount--;

					if ( fgHardStallResetDelayTimerCount == 0 )
					{
						// Hard Stall has timed out, reset MCU
						Reset();							// reset MCU
					}
				}
				else
				{
					// make sure timer is reset for next use
					fgHardStallResetDelayTimerCount = _15S_EVENT_PERIOD;
					BITCLEAR(efTimerEvents, EF_TIMER_HARD_STALL_TIMEOUT);
				}
#endif

				// clear calling event flag
				BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK1);
				//						break;
				continue;									// we have processed something, so restart at the top of the loop
			}



			//					case ST_100mS_TICK2:
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK2))
			{
#ifdef USE_MOVE_SEQ_FSM_TRIGGER
				Trigger1Level(1);						// trigger to allow viewing this event on a scope
#endif
				//						uart_send("\r\n100ms tick 2\r\n");
				if ((efMoveSequenceEvents != NO_EVENTFLAGS) || (IsMoveSequenceComplete() == 0))
					MoveSequenceFSM();
				if ((efMoveSequenceEvents != NO_EVENTFLAGS) || (IsMoveSequenceComplete() == 0) ||(IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))||(IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW)))
					//					MoveSequenceFSM();
#ifdef USE_MOVE_SEQ_FSM_TRIGGER
					Trigger1Level(0);						// trigger to allow viewing this event on a scope
#endif

				// clear calling event flag
				BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK2);
				//						break;
				continue;									// we have processed something, so restart at the top of the loop
			}


			//					case ST_100mS_TICK3:
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK3))
			{
				// run event
#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif
				//						uart_send("\r\n100ms tick 3\r\n");
				if ( ((efMotionPhaseCommands[MOTOR_AZIMUTH] != NO_EVENTFLAGS) || (IsCommandComplete(MOTOR_AZIMUTH) == 0))
						&& (pgeMotionPhase[MOTOR_AZIMUTH] != PHASE_OPEN_LOOP) )
				{
					MotionPhaseFSM(MOTOR_AZIMUTH);
				}

				if ( ((efMotionPhaseCommands[MOTOR_ELEVATION] != NO_EVENTFLAGS) || (IsCommandComplete(MOTOR_ELEVATION) == 0))
						&& (pgeMotionPhase[MOTOR_ELEVATION] != PHASE_OPEN_LOOP) )
				{
					MotionPhaseFSM(MOTOR_ELEVATION);
				}

#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

				// UART tests, intended for inital testing of new hardware
#ifdef UART_TEST
				StartTransmitString(SERIAL_MENU_UART, "This is a test!");
#endif

				// clear calling event flag
				BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK3);
				//						break;
				continue;				// we have processed something, so restart at the top of the loop
			}
			//					case ST_100mS_TICK4:
			if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK4))
			{
				// clear calling event flag
				BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK4);
				//						uart_send("\r\n100ms tick 4\r\n");
				// run event
#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_INCLINOMETER_FEEDBACK)

				// read accelerometer values
				if (ReadInclinometerSample(&pgInclination) == TRUE)							// read accelerometer, calculate 3D angles
				{
					fgbI2CErrorCtr = 0;
					fgbI2CHrdStlCtr  = 0;
					IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);			// add to averaging array, calculate new running average
					//check beyond limits in both Manual and Auto for extreme cases
					if(fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle) >= EXTREME_ANGLES)
					{
						ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
						//						DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
#endif
						Finish_MotionStats(MOTOR_AZIMUTH);
						Transmit("Main.c: Line 1072\n", 18);
						// ==>> does the previous command ever get completed?
						SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
						BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
						ResetMotionFSM();
						//								break;
						continue;
					}

					// EF_MOTION_SENSOR_ACTIVE is set when the PWM is configured for FORWARD or REVERSE, before the start of motion
					// EF_MOTION_SENSOR_ACTIVE is cleared when MotionFSM.c: MotionFSM() enters ST_MOTION_STOPPED for the first time
					// so this will only execute DURING a move, and NOT when stopped.
					if(IS_BITSET(efMotionSensorEvents[MOTOR_AZIMUTH], EF_MOTION_SENSOR_ACTIVE) IS_TRUE)
					{
						if(ptrRAM_SystemParameters->ucTracking_Mode == MODE_TRACKING)
						{
							if(bTrackerDirection == 1)
							{
								if(((fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle-(ptrRTCC_RAM_AppParameters->fSetPoint_AZ))) <= MIN_FLUCT_ANGLE)
										||(pgAngleAverage.fX_Angle >= (ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - MIN_FLUCT_ANGLE)))
								{
									ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									//									DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
#endif
									Finish_MotionStats(MOTOR_AZIMUTH);
									Transmit("Main.c: Line 1098\n", 18);
									// ==>> does the previous command ever get completed?
									SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
									BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
									ResetMotionFSM();
									//											break;
									continue;
								}
							}
							else if(bTrackerDirection == 2)
							{
								if(((fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle-(ptrRTCC_RAM_AppParameters->fSetPoint_AZ))) <= MIN_FLUCT_ANGLE)
										||(pgAngleAverage.fX_Angle <= (ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse + MIN_FLUCT_ANGLE)))
								{
									ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									//									DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
#endif
									Finish_MotionStats(MOTOR_AZIMUTH);
									Transmit("Main.c: Line 1116\n", 18);
									// ==>> does the previous command ever get completed?
									SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
									BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
									ResetMotionFSM();
									//											break;
									continue;
								}
							}
						}                                                                                                                // clear error counter; we are only interested in consecutive errors
						else
						{
							if(MAN_STOW == 1)
							{
								if((pgAngleAverage.fX_Angle <= (ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse + MIN_MANUAL_ANGLE))
										||(pgAngleAverage.fX_Angle >= (ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - MIN_MANUAL_ANGLE))
										||((int)pgAngleAverage.fX_Angle == 0))
								{
									MAN_STOW = 0;
									ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									//									DisplayMessage(SERIAL_MENU_UART,"Reached setpont", NO_WAIT_FOR_DISPLAY);
#endif
									Finish_MotionStats(MOTOR_AZIMUTH);
									Transmit("Main.c: Line 1141\n", 18);
									// ==>> does the previous command ever get completed?
									// mark command as started so we cannot misinterpret completion
									BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
									ResetMotionFSM();
									//											break;
									continue;
								}
							}
							else if(MAN_EAST == 1)
							{
								if((pgAngleAverage.fX_Angle <= (ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse + MIN_MANUAL_ANGLE)))
								{
									MAN_EAST = 0;
									ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									//									DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
#endif
									Finish_MotionStats(MOTOR_AZIMUTH);
									Transmit("Main.c: Line 1158\n", 18);
									// mark command as started so we cannot misinterpret completion
									BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
									ResetMotionFSM();
									//											break;
									continue;
								}
							}
							else if(MAN_WEST == 1)
							{
								if((pgAngleAverage.fX_Angle >= (ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - MIN_MANUAL_ANGLE)))
								{
									MAN_WEST = 0;
									ClearCommandStarted(MOTOR_AZIMUTH);
#if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
									//									DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
#endif
									Finish_MotionStats(MOTOR_AZIMUTH);
									Transmit("Main.c: Line 1175\n", 18);
									BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
									ResetMotionFSM();
									//											break;
									continue;
								}
							}
						}
						BITSET(efMotionSensorEvents[MOTOR_AZIMUTH], EF_MOTION_SENSOR_TICK);
					}
				}
				else
				{
					// bump I2C error counter
					///RuntimeError(INCLINOMETER_ERROR_READ_FAIL);
					if (fgbI2CErrorCtr == MAX_I2C_ERROR_CNT)								// if too many consecutive errors, stop motor (see inclinometer.h)
					{
						fgbI2CErrorCtr = 0;
						ClearCommandStarted(MOTOR_AZIMUTH);
						MAN_EAST = MAN_WEST = 0;
						Finish_MotionStats(MOTOR_AZIMUTH);
						Transmit("Main.c: Line 1197\n", 18);
						// ==>> does the previous command ever get completed?
						SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
						BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
						++fgbI2CHrdStlCtr;
#ifdef MAIN_DEBUG
						DisplayStrSequence(SERIAL_MENU_UART, "Inclinometer fail, soft stall Stop .....");
#endif
						ResetMotionFSM();
					}
					else
					{
						if(fgbI2CHrdStlCtr != MAX_I2C_HrdStal_CNT)
						{
							// inclinometer read error/failure
							++fgbI2CErrorCtr;
							//									I2CReset();
#ifdef MAIN_DEBUG
							DisplayStrSequence(SERIAL_MENU_UART, "Inclinometer Read Failure,i2c reset");
#endif
						}
						else
						{
							fgbI2CHrdStlCtr = 0;
							fgbI2CErrorCtr =0;
							//????
							//BITSET(efMotionResultEvents[MOTOR_AZIMUTH], EF_RESULT_HARD_STALL);
#ifdef MAIN_DEBUG
							DisplayStrSequence(SERIAL_MENU_UART, "Inclinometer fail,hard stall Stop .....");
#endif
						}
					}
				}

#endif	//  defined(USE_SINGLE_POLAR_AXIS) && defined(USE_INCLINOMETER_FEEDBACK)

#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif
				//					break;
				continue;				// we have processed something, so restart at the top of the loop
			}

		}
    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure LSE Drive Capability
//  */
//  HAL_PWR_EnableBkUpAccess();
//  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
//                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
//                              |RCC_OSCILLATORTYPE_LSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 10;
//  RCC_OscInitStruct.PLL.PLLP = 2;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  RCC_OscInitStruct.PLL.PLLR = 1;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_PCLK3;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Enable the SYSCFG APB clock
//  */
//  __HAL_RCC_CRS_CLK_ENABLE();
//
//  /** Configures CRS
//  */
//  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
//  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
//  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
//  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
//  RCC_CRSInitStruct.ErrorLimitValue = 34;
//  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
//
//  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
//}
/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_14B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x30909DEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
  // Enable access to the RTC and Backup registers
//  HAL_PWR_EnableBkUpAccess();

  // Check if RTC has valid data
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
  {
      // RTC data is not valid, initialize RTC with default values
      InitializeRTC();
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
  }
  /* USER CODE END Check_RTC_BKUP */
//  InitializeRTC();
  /** Enable the TimeStamp
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  /* USER CODE END RTC_Init 2 */

}

void InitializeRTC(void)
{
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};

	  sTime.Hours = 0x13;
	  sTime.Minutes = 0x30;
	  sTime.Seconds = 0x00;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0x07;
	  sDate.Year = 0x24;
	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000000 - 1; //500ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1953;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 160-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1; // 1us
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
	  /* Starting Error */
	  Error_Handler();
  }
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM6_Init(uint16_t Period)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 160-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = Period;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(uint16_t Period)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = Period;
  htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim6, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN TIM4_Init 2 */
//  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
//  {
//	  /* Starting Error */
//	  Error_Handler();
//  }
  /* USER CODE END TIM4_Init 2 */

}

///**
//  * @brief USB_OTG_FS Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USB_OTG_FS_PCD_Init(void)
//{
//
//  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
//
//  /* USER CODE END USB_OTG_FS_Init 0 */
//
//  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
//
//  /* USER CODE END USB_OTG_FS_Init 1 */
//  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
//  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
//  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
//  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
//  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
//  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
//  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
//  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
//
//  /* USER CODE END USB_OTG_FS_Init 2 */
//
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRIVE_DIAG_Pin|DRIVE_DRVOFF_Pin|DRIVE_nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Wifi_Enable_GPIO_Port, Wifi_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ACCEL_INT1_Pin|ACCEL_INT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, DRIVE_SR_Pin|DRIVE_ITRIP_Pin|DRIVE_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Flash_Write_Prot_Pin|Flash_Hold_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRIVE_nFAULT_Pin */
  GPIO_InitStruct.Pin = DRIVE_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DRIVE_nFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DRIVE_DIAG_Pin DRIVE_DRVOFF_Pin DRIVE_nSLEEP_Pin */
  GPIO_InitStruct.Pin = /*DRIVE_DIAG_Pin|*/DRIVE_DRVOFF_Pin|DRIVE_nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Wind_Input_Pin */
  GPIO_InitStruct.Pin = Wind_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Wind_Input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Wifi_Enable_Pin */
  GPIO_InitStruct.Pin = Wifi_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Wifi_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACCEL_INT1_Pin ACCEL_INT2_Pin */
  GPIO_InitStruct.Pin = ACCEL_INT1_Pin|ACCEL_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DRIVE_SR_Pin DRIVE_ITRIP_Pin DRIVE_MODE_Pin */
  /*GPIO_InitStruct.Pin = DRIVE_SR_Pin|DRIVE_ITRIP_Pin|DRIVE_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);*/

  /*Configure GPIO pins : Flash_Write_Prot_Pin Flash_Hold_Pin */
	GPIO_InitStruct.Pin = Flash_Write_Prot_Pin|Flash_SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = Flash_Hold_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Flash_SPI2_CS_GPIO_Port, Flash_SPI2_CS_Pin, GPIO_PIN_SET);
	/*Configure GPIO pins : PB_Manual_Pin PB_Auto_Pin */
	GPIO_InitStruct.Pin = PB_Manual_Pin|PB_Auto_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Drv8245 CS Pin */
	GPIO_InitStruct.Pin = Wifi_SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Wifi_SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(Wifi_SPI1_CS_GPIO_Port, Wifi_SPI1_CS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOD, Flash_Write_Prot_Pin, GPIO_PIN_RESET);

	/*	GPIO_InitStruct.Pin = DRIVE_PH_IN2_TIM_CH1_Pin | DRIVE_EN_IN1_TIM_CH2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/
	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

void InitializeParameretes(void)
{

	//	ret_code_t err_code;

	char tx_message[] = "\r\nHello Tracker serial Enabled\r\n";


	// Added the below RTC Code for Testing//
/*	RTCC_DATE_TIME CurrentDateTime;
	UINT8 reg;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;*/
	char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	// get, display current date and time

	ADDBUILDDateTime(szfnDisplayStr);

/*
	if (WriteRTCCRegister(DS3232_REG_HOURS, 14) IS_FALSE)
	{
		DisplayMessage("WriteRTCCRegister Failure", WAIT_FOR_DISPLAY);
	}

	memset(szfnDisplayStr, 0 , sizeof(szfnDisplayStr));

	if (ReadRTCCRegister(DS3232_REG_HOURS, &reg) IS_FALSE)
	{
		DisplayMessage("ReadRTCCRegister Failure", WAIT_FOR_DISPLAY);
	}

	if (WriteRTCCRegister(DS3232_REG_SECONDS, 39) IS_FALSE)
	{
		DisplayMessage("WriteRTCCRegister Failure", WAIT_FOR_DISPLAY);
	}

	if(ReadRTCCDateTime(ptrDateTime) != TRUE)
	{
		DisplayMessage("Unable to Read RTCC\r\n", WAIT_FOR_DISPLAY);
	}
	IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
	DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);
*/

	/**********************Tracker Task added here*****************/

	/*****************************************************************************************/
	/*************************************SPI Initialize*************************************/
	/****************************************************************************************/
	// initialize SST25 SPI flash
	InitSystemParameterTable();

	DisplayMessage( "", WAIT_FOR_DISPLAY);
	DisplayMessage( "", WAIT_FOR_DISPLAY);

#ifdef USE_MMA8452Q_INCLINOMETER
	if (MMA845x_Init() != TRUE)
	{
		uart_send("\r\nMMA845 Init Failed.\r\n");
#ifdef MAIN_DEBUG
				DisplayMessage( "Failed Inclinometer Initialization ", WAIT_FOR_DISPLAY);
#endif
	}
//	else
//		uart_send("\r\nMMA845 Init Success.\r\n");
#endif


	MotionSensor_Init();			// initialize T3, used for motion timing
//	uart_send("\r\nMotion Sensor Init.\r\n");

#ifdef USE_DS3232_RTCC
	// init the MCU RAM copy of the RTCC RAM parameter tables
	if(InitRTCCRAMParameterTable() != MEMORY_INITIALIZED)			// re-initialize tables (orientation will be 0, 0)
	{
//		uart_send("\r\nRTCC RAM Init Failed.\r\n");
#ifdef MAIN_DEBUG
				DisplayMessage("Failed RTCC NV RAM Initialization", WAIT_FOR_DISPLAY);
#endif
	}
	else
	{
		//		uart_send("\r\nRTCC RAM Init Success.\r\n");
		IGNORE_RETURN_VALUE InitRTCCRAMParameterTable();
	}
#endif

	Init_MotionStats(MOTOR_AZIMUTH);
	Init_MotionStats(MOTOR_ELEVATION);
//	uart_send("\r\nMotion stats Init.\r\n");

#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	// initialize the Polar Axis Move table; stores orientation at the end of each move, for debugging/analysis ONLY
	ClearPolarAxisMoveTable();
//	uart_send("\r\nPolar axis cleared.\r\n");
#endif	// defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)

#ifdef USE_AZIMUTH
	PWM_Init(MOTOR_AZIMUTH);
//	uart_send("\r\nPWM Timer loop Init.\r\n");
#endif

#ifdef USE_ELEVATION
	PWM_Init(MOTOR_ELEVATION);
#endif

	/********************************************************************************************/
	/*************************************Initialize FSMs*************************************/
	/******************************************************************************************/

	// these initial calls take the FSMs OUT of the  xxxx_INIT states
#ifdef USE_PCA9554_IO				// enable in config.h ONLY if PCA9554 hardware is present
	Input_Debounce_FSM(TRUE);		// input switch debounce handling, force Reset
#endif		// USE_PCA9554_IO

#ifdef SWITCH_INIT
	Switch_Init();
	Switch_processing();
#endif
	//	ButtonProcessingFSM();

	MotionFSM(MOTOR_AZIMUTH);
	MotionFSM(MOTOR_ELEVATION);
//	uart_send("\r\nMotion FSM\r\n");

	MotionPhaseFSM(MOTOR_AZIMUTH);
	MotionPhaseFSM(MOTOR_ELEVATION);
//	uart_send("\r\nMotionPhase\r\n");
	IGNORE_RETURN_VALUE PanelPositionFSM(&fgOrientation);					// first call restores previously stored orientation to position counters; argument is a dummy

	MoveSequenceFSM();
//	uart_send("\r\nMove Sequence started\r\n");
	// ********************************
	//		Initialize Zigbee
	// ********************************
#ifdef CC2530
	//read short address
	//Read_Address();
#endif

	// ********************************
	//	Initialize, Display Inclination
	// ********************************
#ifdef USE_MMA8452Q_INCLINOMETER
	// read inclinometer and fill averaging array before allowing ANY motion
	if (Init_InclinometerSampleAveraging() IS_TRUE)
	{
//		uart_send("\r\nInclinometer sample success\r\n");
		// read and display inclinometer angles
		if (ReadInclinometerSample(&pgInclination) == TRUE)							// read accelerometer, calculate 3D angles
		{
			IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);// add to averaging array, calculate new running average
//			uart_send("\r\nRead Inclinometer success\r\n");

#ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
			//			DisplayMessage( "Initial Inclination:", WAIT_FOR_DISPLAY);
			IGNORE_RETURN_VALUE FormatAverageInclination(szfnDisplayStr);			// format current average values for display
			//			DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);
#endif
		}
		else
		{
//			uart_send("\r\nRead Inclinometer Failed\r\n");
#ifdef MAIN_DEBUG
						DisplayMessage("Failed Inclinometer Read ", WAIT_FOR_DISPLAY);
#endif
		}

		// initialze previous position to stored AVERAGED value, before allowing ANY motion
		fglPreviousPosition[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
		CurrentPosition_Set(MOTOR_AZIMUTH, fglPreviousPosition[MOTOR_AZIMUTH]);
	}
	else
	{
//		uart_send("\r\nInclinometer sample Failed\r\n");
#ifdef MAIN_DEBUG
				DisplayMessage( "Failed Inclinometer Read ", WAIT_FOR_DISPLAY);
#endif
	}
#endif

	ptrRAM_SystemParameters->ucTracking_Mode = MODE_TRACKING;
	// ********************************
	//	Startup Tracking Mode
	// ********************************

	switch(ptrRAM_SystemParameters->ucTracking_Mode)
	{
	case MODE_MANUAL:
//		uart_send("\r\nManual Mode\r\n");
		break;

	case MODE_TRACKING:
//		uart_send("\r\nTracking mode\r\n");
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
		BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);		// OPEN will be processed AFTER CLOSED, has the effect of push and release
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK);
		break;

	case MODE_NIGHT_STOW:
		break;

	case MODE_WIND_STOW:
		break;

	default:
//		uart_send("\r\nMode Error\r\n");
		//			RuntimeError(MENU_FSM_ERROR_OUT_OF_RANGE_PARAMETER);
		break;
	}

	// ********************************
	//		Enable Watchdog
	// ********************************
	//	EnableWDT();

	// ********************************
	//	Update Serial Output Mode
	// ********************************

	// update the serial output mode from SER_MODE_MENU (initialized above) to the value stored in the system flash structure
	if (ptrRAM_SystemParameters->eSerialOutputMode == SER_MODE_REMOTE)
	{
		//		eSerialOutputMode = SER_MODE_REMOTE;			// update program global copy
//		uart_send("\r\nRemote mode\r\n");
		//		ChangeSerialBaudRate(SERIAL_REMOTE_UART, DESIRED_REMOTE_BAUDRATE);
	}


	//	RTC Testing End//

	// DRV8245 Testing//

//	  DWT_Init();
//	  WakeUpDrv8245();
//	  SetPins();
//	  EnableDrv8245();

//	  while(1)
//	  {
//		  SetMotor(1);
//		  HAL_Delay(2000);
//		  SetMotor(0);
//		  HAL_Delay(2000);
//	  }

	// DRV8245 Testing End//

	// LIS2HH12 Testing//
//    uint8_t whoAmI;
//    LIS2HH12TR_ReadData(0x0F, &whoAmI, 1);
//
//    // Read various registers
//    uint8_t ctrl1, ctrl2, ctrl3, status, xOutL, xOutH, yOutL, yOutH, zOutL, zOutH;
//
//    LIS2HH12TR_ReadData(0x20, &ctrl1, 1); // Read CTRL1 register
//    LIS2HH12TR_ReadData(0x21, &ctrl2, 1); // Read CTRL2 register
//    LIS2HH12TR_ReadData(0x22, &ctrl3, 1); // Read CTRL3 register
//    LIS2HH12TR_ReadData(0x27, &status, 1); // Read STATUS register
//    LIS2HH12TR_ReadData(0x28, &xOutL, 1); // Read X-axis low byte
//    LIS2HH12TR_ReadData(0x29, &xOutH, 1); // Read X-axis high byte
//    LIS2HH12TR_ReadData(0x2A, &yOutL, 1); // Read Y-axis low byte
//    LIS2HH12TR_ReadData(0x2B, &yOutH, 1); // Read Y-axis high byte
//    LIS2HH12TR_ReadData(0x2C, &zOutL, 1); // Read Z-axis low byte
//    LIS2HH12TR_ReadData(0x2D, &zOutH, 1); // Read Z-axis high byte

//		int16_t Data;
//		LIS2HH12_Init();
//		LIS2HH12_ReadAccData(&Data);
	// LIS2HH12 Testing End//

//	uart_send("\r\nTracker Init End.\r\n");
}

void delay_us(uint32_t  us)
{
    uint32_t start_count = __HAL_TIM_GET_COUNTER(&htim4); // Get the current counter value

    while ((__HAL_TIM_GET_COUNTER(&htim4) - start_count) < us); // Wait until us microseconds have passed
}

//void refresh(void) {
//    if (HAL_WWDG_Refresh(&hwwdg) != HAL_OK)
//    {
//      Error_Handler();
//    }
//}
//
//void refresh_delay(uint8_t seconds) {
//	for (uint8_t i = 0; i < (seconds * 1000);) {
//		HAL_Delay(100);
//		refresh();
//		i = i+100;
//	}
//}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  hiwdg.Init.Reload = 2500-1;
  hiwdg.Init.EWI = 0;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */
     /*  Default WWDG Configuration:
      1] Set WWDG counter to 0x7F  and window to 0x50
      2] Set Prescaler to WWDG_PRESCALER_64

      Timing calculation:
      a) WWDG clock counter period (in ms) = (4096 * WWDG_PRESCALER_64) / (PCLK1 / 1000)
                                           = 1,638 ms
      b) WWDG timeout (in ms) = (0x7F + 1) * 1,638
                              ~= 209,71 ms
      => After refresh, WWDG will expires after 209,71 ms and generate reset if
      counter is not reloaded.
      c) Time to enter inside window
      Window timeout (in ms) = (127 - 80 + 1) * 1,638
                             = 78,64 ms */
  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_64;
  hwwdg.Init.Window = 0x50;
  hwwdg.Init.Counter = 0x7F;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

}

/**
  * @brief  Timeout calculation function.
  *         This function calculates any timeout related to
  *         WWDG with given prescaler and system clock.
  * @param  timevalue: period in term of WWDG counter cycle.
  * @retval None
  */
static uint32_t TimeoutCalculation(uint32_t timevalue)
{
  uint32_t timeoutvalue = 0;
  uint32_t pclk1 = 0;
  uint32_t wdgtb = 0;

  /* considering APB divider is still 1, use HCLK value */
  pclk1 = HAL_RCC_GetPCLK1Freq();

  /* get prescaler */
  wdgtb = (1 << ((hwwdg.Init.Prescaler) >> WWDG_CFR_WDGTB_Pos)); /* 2^WDGTB[1:0] */

  /* calculate timeout */
  timeoutvalue = ((4096 * wdgtb * timevalue) / (pclk1 / 1000));

  return timeoutvalue;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
//	__disable_irq();
	NVIC_SystemReset();
	/* Turn LED3 on */
	while (1);
  /* USER CODE END Error_Handler_Debug */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

    timer_count++; // Increment the count variable
    if (timer_count >= 5000) // 5 ms has passed
    {
    	// check for 5mS event overrun
    	if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
    	{
    //		RuntimeError(TIMER_ERROR_5MS_TICK_OVERRUN);
    	}

    	BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT);
        timer_count = 0; // Reset the count
    }
/*	// check for 5mS event overrun
	if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
	{
//		RuntimeError(TIMER_ERROR_5MS_TICK_OVERRUN);
	}

	BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT);*/

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
