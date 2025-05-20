// *************************************************************************************************
//										M a i n . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	main() initialization and scheduler
//
// *************************************************************************************************
// *******************************************************************************
/*
FileName:       main.c
Dependencies:   See includes
Processor:      PIC32MX

Complier:       Microchip MPLAB XC32 v1.6 or higher
*/
// *****************************************************************************
// *****************************************************************************
// Section: Includes
// *****************************************************************************
// *****************************************************************************
#include <GenericTypeDefs.h>
#include <math.h>
#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14
#include <legacy\int_legacy.h>			// required for various interrupt handlers
#include <legacy\int_3xx_4xx_legacy.h>	// required for various interrupt handlers

#define	DEFINE_GLOBALS
// NOTE: all header files that define global variables MUST be included here, even if the globals are not otherwise used in this file
#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"
#include "SerialPort.h"
#include "SerialDisplay.h"		// display functions for menus
#include "MenuFSM.h"
//#include "SST25VF016.h"		// SPI Flash function definitions

//#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
#include "EventFlags.h"			// event flag definitions and globals
//#include "init.h"
//#include "ADCRead.h"
#include "Debounce.h"

#include "I2CBus.h"
#include "DS3232.h"				// Real Time Clock
#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"              // MMA845xQ definitions
	#include "Inclinometer.h"
#endif

#include "TimeDelay.h"

#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"		// Sun Position FSM - operating modes
#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"
#include "ButtonProcessingFSM.h" // Button and user input processing
#include "RxMessage.h"

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
#include "ADC10Read.h"
#include "BMS.h"
#include "CoordTranslate.h"
#include "Zigbee.h"
// MCU configuration bits
//lint -e766	error 766: (Info -- Header file 'MCUConfigurationBits.h' not used in module 'main.c')
#include "MCUConfigurationBits.h"
//lint +e776

#undef	DEFINE_GLOBALS

//----------------------------------------------------------------------
// Definitions
//----------------------------------------------------------------------

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


//-----------------------------------------------------------------------------
// Static and File-Local Variables
//-----------------------------------------------------------------------------

// This is a software counter used to time slower system events, such as button polling, etc.
FILE_GLOBAL_INIT unsigned int fg100mSTimerCount1 = _100MS_EVENT_PERIOD;
FILE_GLOBAL_INIT unsigned int fg100mSTimerCount2 = _100MS_EVENT_PERIOD;
FILE_GLOBAL_INIT unsigned int fg100mSTimerCount3 = _100MS_EVENT_PERIOD;
FILE_GLOBAL_INIT unsigned int fg100mSTimerCount4 = _100MS_EVENT_PERIOD;

FILE_GLOBAL_INIT BYTE fgbI2CErrorCtr = 0;	// Track I2C errors when reading Inclinometer
FILE_GLOBAL_INIT BYTE fgbI2CHrdStlCtr = 0;	// Track I2C errors when reading Inclinometer

PRIVATE_INIT float  fgdefferenceAngle = 0.0;
#define MIN_FLUCT_ANGLE     0.5
#define MIN_MANUAL_ANGLE     2

// these are just dummy structures passed to PanelPositionFSM() for initialization
PRIVATE SmartTrakOrientation fgOrientation;

// Let compile time pre-processor calculate the CORE_TICK_PERIOD
#define TOGGLES_PER_SEC			200
#define CORE_TICK_RATE	        (GetSystemClock()/2/TOGGLES_PER_SEC)

// *****************************************************************************
// from C:\Program Files\Microchip\xc32\v1.20\pic32mx\include\peripheral\legacy\int_1xx_2xx_legacy.h:
// *****************************************************************************
// these macros only exist in the LEGACY header files.. why?
/*
#define mCTClearIntFlag()                   (IFS0CLR = _IFS0_CTIF_MASK)
#define mCTGetIntFlag()                     (IFS0bits.CTIF)
#define mCTIntEnable(enable)                (IEC0CLR = _IEC0_CTIE_MASK, IEC0SET = ((enable) << _IEC0_CTIE_POSITION))
#define mCTGetIntEnable()                   (IEC0bits.CTIE)
#define mCTSetIntPriority(priority)         (IPC0CLR = _IPC0_CTIP_MASK, IPC0SET = ((priority) << _IPC0_CTIP_POSITION))
#define mCTGetIntPriority()                 (IPC0bits.CTIP)
#define mCTSetIntSubPriority(subPriority)   (IPC0CLR = _IPC0_CTIS_MASK, IPC0SET = ((subPriority) << _IPC0_CTIS_POSITION))
#define mCTGetIntSubPriority()              (IPC0bits.CTIS)
 */
// *****************************************************************************

// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constant Data
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// Section: Code
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// int main(void)
// *****************************************************************************
int main(void)
{

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
        int dly10=0;
	#ifndef _lint		// too many complex lint errors here!
		// Configure the device for maximum performance, but do not change the PBDIV clock divisor.
		// Given the options, this function will change the program Flash wait states,
		// RAM wait state and enable prefetch cache, but will not change the PBDIV.
		// The PBDIV value is already set via the pragma FPBDIV option above.
		IGNORE_RETURN_VALUE SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

		// configure the core timer roll-over rate (100msec)
		OpenCoreTimer(CORE_TICK_RATE);

		// set up the core timer interrupt with a prioirty of 2 and zero sub-priority
		mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));
	#endif	// _lint

	InitTriggerPins();
	
	#ifdef USE_ELEVATION_ON_OFF_LINEAR_DRIVE	// use linear actuator, ON/OFF relay drive
		InitJ9Pins();
	#endif

	InitializeSerialPort(SERIAL_MENU_UART, DESIRED_MENU_BAUDRATE);
	eSerialOutputMode = SER_MODE_MENU;			// initialze file global copy for use by serial display functions

        InitializeSerialPort(SERIAL_REMOTE_UART, DESIRED_MENU_BAUDRATE);
	eSerialOutputMode = SER_MODE_MENU;			// initialze file global copy for use by serial display functions

	#ifdef USE_REMOTE_COMMANDS
		RxMessageFSM(SERIAL_REMOTE_UART);			// initialize FSM
	#endif

	// ********************************
	//		Initialize I2C Bus
	// ********************************
	if (InitializeI2CBus() IS_NOT TRUE)
	{
                #ifdef MAIN_DEBUG
		DisplayMessage(SERIAL_MENU_UART, "Failed I2C Bus Initialization", WAIT_FOR_DISPLAY);
                #endif
	}

	#ifndef _lint		// too many complex lint errors here!
		// configure for multi-vectored mode
		INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

		// enable interrupts
		IGNORE_RETURN_VALUE INTEnableInterrupts();

		// enable device multi-vector interrupts
		INTEnableSystemMultiVectoredInt();
	#endif	// _lint

	// ********************************
	//		Initialize SPI Flash
	// ********************************
	// initialize SST25 SPI flash
                InitSystemParameterTable();
 /*
	switch(InitSystemParameterTable())
	{
		case MEMORY_TIMEOUT:
                        #ifdef MENU_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "\x1B[2J\x1B[0;0H==SPI FLASH Memory Timeout (Press a key to continue)", WAIT_FOR_DISPLAY);
                        #endif
			while (AnyRxDataAvailable(SERIAL_MENU_UART) IS_FALSE)			// wait for any keystroke
				BLOCKING_DELAY;
			break;
		case MEMORY_NOT_PRESENT:
                        #ifdef MENU_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "\x1B[2J\x1B[0;0H==SPI FLASH not found (Press a key to continue)", WAIT_FOR_DISPLAY);
                        #endif
			while (AnyRxDataAvailable(SERIAL_MENU_UART) IS_FALSE)
				BLOCKING_DELAY;
			break;
		case MEMORY_CHECKSUM_ERROR:
                        #ifdef MENU_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "\x1B[2J\x1B[0;0H==SPI FLASH Checksum Error (Press a key to continue)", WAIT_FOR_DISPLAY);
                        #endif
			while (AnyRxDataAvailable(SERIAL_MENU_UART) IS_FALSE)
				BLOCKING_DELAY;
			break;
		case MEMORY_INITIALIZED:
			break;

		default:
                        #ifdef MENU_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "\x1B[2J\x1B[0;0H==SPI FLASH unknown error (Press a key to continue)", WAIT_FOR_DISPLAY);
                        #endif
			while (AnyRxDataAvailable(SERIAL_MENU_UART) IS_FALSE)
				BLOCKING_DELAY;
	}*/
        DisplayMessage(SERIAL_MENU_UART, "Hello", WAIT_FOR_DISPLAY);
        DisplayMessage(SERIAL_REMOTE_UART, "", WAIT_FOR_DISPLAY);
      //  SendMBMessage(SERIAL_MENU_UART,"n1",2, WAIT_FOR_DISPLAY);

      //ADC Initialisation
       IGNORE_RETURN_VALUE InitADC10();
       Intial_calc_adc();
	// ********************************
	//		Initialize Peripherals
	// ********************************
	#ifdef USE_MMA8452Q_INCLINOMETER
		if (MMA845x_Init() IS_NOT TRUE)
		{
                    #ifdef MAIN_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "Failed Inclinometer Initialization ", WAIT_FOR_DISPLAY);
                    #endif
		}
	#endif

	MotionSensor_Init();			// initialize T3, used for motion timing

	#ifdef USE_DS3232_RTCC
		// init the MCU RAM copy of the RTCC RAM parameter tables
		if(InitRTCCRAMParameterTable() IS_NOT MEMORY_INITIALIZED)			// re-initialize tables (orientation will be 0, 0)
		{
                        #ifdef MAIN_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "Failed RTCC NV RAM Initialization", WAIT_FOR_DISPLAY);
                        #endif
		}

		IGNORE_RETURN_VALUE InitRTCCRAMParameterTable();
	#endif

	Init_MotionStats(MOTOR_AZIMUTH);
	Init_MotionStats(MOTOR_ELEVATION);

	#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
		// initialize the Polar Axis Move table; stores orientation at the end of each move, for debugging/analysis ONLY
		ClearPolarAxisMoveTable();
	#endif	// defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)

	#ifdef USE_AZIMUTH
	PWM_Init(MOTOR_AZIMUTH);
	#endif

	#ifdef USE_ELEVATION
	PWM_Init(MOTOR_ELEVATION);
	#endif

	// ********************************
	//		Initialize FSMs
	// ********************************
	// these initial calls take the FSMs OUT of the  xxxx_INIT states
	#ifdef USE_PCA9554_IO				// enable in config.h ONLY if PCA9554 hardware is present
		Input_Debounce_FSM(TRUE);		// input switch debounce handling, force Reset
	#endif		// USE_PCA9554_IO

        #ifdef SWITCH_INIT
        Switch_Init();
        Switch_processing();
        #endif
	ButtonProcessingFSM();

	MotionFSM(MOTOR_AZIMUTH);
	MotionFSM(MOTOR_ELEVATION);

	MotionPhaseFSM(MOTOR_AZIMUTH);
	MotionPhaseFSM(MOTOR_ELEVATION);

	IGNORE_RETURN_VALUE PanelPositionFSM(&fgOrientation);					// first call restores previously stored orientation to position counters; argument is a dummy

	MoveSequenceFSM();
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
			// read and display inclinometer angles
			if (ReadInclinometerSample(&pgInclination) IS TRUE)							// read accelerometer, calculate 3D angles
			{
                                IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);// add to averaging array, calculate new running average
                                #ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
				DisplayMessage(SERIAL_MENU_UART, "Initial Inclination:", WAIT_FOR_DISPLAY);							
				IGNORE_RETURN_VALUE FormatAverageInclination(szfnDisplayStr);			// format current average values for display
				DisplayMessage(SERIAL_MENU_UART, szfnDisplayStr, WAIT_FOR_DISPLAY);
                                #endif
			}
			else
			{
                            #ifdef MAIN_DEBUG
				DisplayMessage(SERIAL_MENU_UART, "Failed Inclinometer Read ", WAIT_FOR_DISPLAY);
                            #endif
			}

			// initialze previous position to stored AVERAGED value, before allowing ANY motion
			fglPreviousPosition[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
			CurrentPosition_Set(MOTOR_AZIMUTH, fglPreviousPosition[MOTOR_AZIMUTH]);
		}
		else
		{
                    #ifdef MAIN_DEBUG
			DisplayMessage(SERIAL_MENU_UART, "Failed Inclinometer Read ", WAIT_FOR_DISPLAY);
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
			break;

		case MODE_TRACKING:
			SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
			BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
			BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);		// OPEN will be processed AFTER CLOSED, has the effect of push and release
//			BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK);
			break;

		case MODE_NIGHT_STOW:
			break;

		case MODE_WIND_STOW:
			break;

		default:
//			RuntimeError(MENU_FSM_ERROR_OUT_OF_RANGE_PARAMETER);
			break;
	}

	// ********************************
	//		Enable Watchdog
	// ********************************
	EnableWDT();

	// ********************************
	//	Update Serial Output Mode
	// ********************************

	// update the serial output mode from SER_MODE_MENU (initialized above) to the value stored in the system flash structure
	if (ptrRAM_SystemParameters->eSerialOutputMode IS SER_MODE_REMOTE)
	{
		eSerialOutputMode = SER_MODE_REMOTE;			// update program global copy
		ChangeSerialBaudRate(SERIAL_REMOTE_UART, DESIRED_REMOTE_BAUDRATE);
	}

	// ************************************************************************
	//							main() loop / scheduler
	// ************************************************************************
	for(;;)
		{
		// This is the main software loop.  The watchdog timer is cleared and the flags are polled for various event handlers.
		// All events are scheduled using timer interrupts and software counters.

		ClearWDT();		// Clear the watchdog timer


		//-----------------------------------------------------------------
		//	Motion Sensor Tick handler executes as required
		//-----------------------------------------------------------------
		// for any 'speed' greater than MotionProfileSpeedAndPWM[row 39] results in a MSI tick FASTER than 5mS, so we need to process this FIRST
		// see spreadsheet SmartTrak Solar Tracker Motion Calculations A02.XLS

		// timing triggers in MotionSensor.c

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

		//-----------------------------------------------------------------
		//	ADC Tick handler executes as required
		//-----------------------------------------------------------------
		// the ADC interrupt is supposed to occur at a 6KHz rate, but it does not (currently) do so

		if (IS_BITSET(efADCEvents, EF_ADC_CONVERSION_DONE))
			{
//			ADC_Tick();				// processes and interprets ADC values

//			continue;				// we have processed something, so restart at the top of the loop
			}


		//-----------------------------------------------------------------
		// Generate Sub-timer Event Flags on 5mS tick
		//-----------------------------------------------------------------

		if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
			{
			// clear event flag (5mS event overrun is handled in the 5mS timer interrupt)
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

			// each of these states occurs once every 25mS, so we can distribute slower events across different 5mS ticks
			// there is no need for 5mS tick counters, because this is a 5 state FSM, called every 5mS
			switch (e5mSTickState)
				{
				//-------------------------------
				//			5mS Tick 1
				//-------------------------------
				case ST_5mS_TICK1:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1))
						{
						RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
						}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1);
					e5mSTickState = ST_5mS_TICK2;			// bump state for next tick
					break;

				//-------------------------------
				//			5mS Tick 2
				//-------------------------------
				case ST_5mS_TICK2:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK2))
						{
						RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
						}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK2);
					e5mSTickState = ST_5mS_TICK3;			// bump state for next tick
					break;

				//-------------------------------
				//			5mS Tick 3
				//-------------------------------
				case ST_5mS_TICK3:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3))
						{
						RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
						}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3);
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
							RuntimeError(MAIN_ERROR_25MS_MOTION_TICK_OVERRUN);
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

					//-------------------------------
					//	25mS Elevation Motion Timer
					//-------------------------------
					if (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER))
						{
						// Elevation motion timer is enabled
						// check for 25mS Motion Timer event overrun
						if (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK))
							{
							RuntimeError(MAIN_ERROR_25MS_MOTION_TICK_OVERRUN);
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
						RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
						}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK4);

					e5mSTickState = ST_5mS_TICK5;			// bump state for next tick
					break;

				//-------------------------------
				//			5mS Tick 5
				//-------------------------------
				case ST_5mS_TICK5:
					// check for 25mS event overrun
					if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5))
						{
						RuntimeError(MAIN_ERROR_25MS_TICK_OVERRUN);
						}

					BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5);
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
				if (pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH] IS_NOT 0)
				{
					--pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH];

					// check for end of count
					if (pgwFeedbackSimulatorTickCtr[MOTOR_AZIMUTH] IS 0)
					{
						// counter has expired, so reload the simulated Motion Sensor tick counter
						MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);

						// create a Motion Sensor Feedback pulse
						SetFeedbackSimulator(MOTOR_AZIMUTH);					// feedback pin HIGH
						// Motion Sensor interrupt will occur HERE
						ClearFeedbackSimulator(MOTOR_AZIMUTH);					// feedback pin LOW
					}
				}

				if (pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION] IS_NOT 0)
				{
					--pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION];

					// check for end of count
					if (pgwFeedbackSimulatorTickCtr[MOTOR_ELEVATION] IS 0)
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

			if ( ((efMotionResultEvents[MOTOR_AZIMUTH] IS_NOT NO_EVENTFLAGS) /* OR ((efSwitchEvents & EF_SWITCH_UP_AZ_EVENTS_MASK) IS_NOT (WORD)0) */)
				AND (pgeMotionPhase[MOTOR_AZIMUTH] IS_NOT PHASE_OPEN_LOOP) )
				{
				MotionPhaseFSM(MOTOR_AZIMUTH);				// event(s) to be processed, so run the Move Command FSM without waiting for the timer
				}

			if ( ((efMotionResultEvents[MOTOR_ELEVATION] IS_NOT NO_EVENTFLAGS) /* OR ((efSwitchEvents & EF_SWITCH_UP_EL_EVENTS_MASK) IS_NOT (WORD)0) */)
				AND (pgeMotionPhase[MOTOR_ELEVATION] IS_NOT PHASE_OPEN_LOOP) )

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
			if ( ((efMotionEvents[MOTOR_AZIMUTH] IS_NOT NO_EVENTFLAGS) /*OR (IsMotionComplete(MOTOR_AZIMUTH) IS_FALSE)*/ /*OR (IS_BITSET(efTimerEvents, EF_TIMER_MOTION_25MS_TICK))*/)		// <== kludge?
					AND (pgeMotionPhase[MOTOR_AZIMUTH] IS_NOT PHASE_OPEN_LOOP) )

				{
				MotionFSM(MOTOR_AZIMUTH);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

			if ( ((efMotionEvents[MOTOR_ELEVATION] IS_NOT NO_EVENTFLAGS)/* OR (IsMotionComplete(MOTOR_ELEVATION) IS_FALSE)*/ /* OR (IS_BITSET(efTimerEvents, EF_TIMER_MOTION_25MS_TICK))*/)		// <== kludge?
					AND (pgeMotionPhase[MOTOR_ELEVATION] IS_NOT PHASE_OPEN_LOOP) )
				{
				MotionFSM(MOTOR_ELEVATION);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

			}		// end if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
			#ifdef USE_MOTION_FSM_5MS_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
			#endif


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

		if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK1))
			{
			// clear calling event
			BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK1);
			#ifdef USE_MENU_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
			#endif

			#ifdef UART_DUAL_TEST
				/*if (ptrRAM_SystemParameters->eSerialOutputMode IS SER_MODE_MENU)
				{
					MenuFSM(SERIAL_MENU_UART);
				}
				#ifdef USE_REMOTE_COMMANDS
				else if (ptrRAM_SystemParameters->eSerialOutputMode IS SER_MODE_REMOTE)
				{
					RxMessageFSM(SERIAL_REMOTE_UART);
				}
				#endif*/
                                MenuFSM(SERIAL_MENU_UART);
                                #endif
                        #ifdef RXMSG
                            RxMessageFSM(SERIAL_REMOTE_UART);
                        #endif
                        #ifdef BOTH_UARTS
                            eSerialOutputMode = SER_MODE_MENU;
                            //MenuFSM(SERIAL_MENU_UART);
                             MenuFSM(SERIAL_REMOTE_UART);

                            eSerialOutputMode = SER_MODE_REMOTE;
                            RxMessageFSM(SERIAL_REMOTE_UART);
                         #endif
			#ifdef USE_MENU_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
			#endif
			continue;							// we have processed something, so restart at the top of the loop
			}


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

			// check for timer complete - or (ACK!) rolled under
			if ((fg100mSTimerCount1 IS (unsigned int)0) OR (fg100mSTimerCount1 > _100MS_EVENT_PERIOD))
				{
				// no need to check for overrun on 100mS events

				// set event flag
				BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK1);

				// reload timer
				fg100mSTimerCount1 = _100MS_EVENT_PERIOD;
				}
                        /////////////////////////////////////////////////////////////////
			// if the MotionPhase is PHASE_OPEN_LOOP, we are running OpenLoop menus selections, so there is no need to run MotionFSM()
			// if the (efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK) bit is set, we are ALREADY calling the MotionFSM() on a timed basis on 25mS Tick4, so we can skip this call...  16 Apr 13 <sek>

			#ifdef USE_MOTION_FSM_25MS_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
			#endif

			if ((pgeMotionPhase[MOTOR_AZIMUTH] IS_NOT PHASE_OPEN_LOOP) AND (IS_BITCLEAR(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK)))
				{
				// run Motion Control FSM on a timed basis
				MotionFSM(MOTOR_AZIMUTH);
				}

			#ifdef USE_ELEVATION
				// if the MotionPhase is PHASE_OPEN_LOOP, we are running OpenLoop menus selections, so there is no need to run MotionFSM()
				// if the (efMotionTimerEvents[MOTOR_AZIMUTH], MOTOR_ELEVATION) bit is set, we are ALREADY calling the MotionFSM() on a timed basis, so we can skip this call...  16 Apr 13 <sek>
				if ((pgeMotionPhase[MOTOR_ELEVATION] IS_NOT PHASE_OPEN_LOOP) AND (IS_BITCLEAR(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK)))
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

			continue;				// we have processed something, so restart at the top of the loop
			}


		//-------------------------------
		//			25mS Tick 3
		//-------------------------------
		if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK3))
			{
			// run Menu FSM on a timed basis
			#ifndef UART_TEST
//				MenuFSM(SERIAL_MENU_UART);
			#endif

			// bump 100mS tick counter
			fg100mSTimerCount2--;

			// check for timer complete - or (ACK!) rolled under
			if ((fg100mSTimerCount2 IS (unsigned int)0) OR (fg100mSTimerCount2 > _100MS_EVENT_PERIOD))
				{
				// no need to check for overrun on 100mS events

				// set event flag
				BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK2);

				// reload timer
				fg100mSTimerCount2 = _100MS_EVENT_PERIOD;
				}


			// clear calling event
			BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK3);

			continue;								// we have processed something, so restart at the top of the loop
													// this prevents processing EF_SCHED_25MS_TICK3 and EF_SCHED_100MS_TICK1 on the same 5mS tick
			}

		//-------------------------------
		//			25mS Tick 4
		//		Motion Timer Processing
		//-------------------------------
		if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK4))
			{
			// bump 100mS tick counter
			fg100mSTimerCount3--;

			// check for timer complete - or (ACK!) rolled under
			if ((fg100mSTimerCount3 IS (unsigned int)0) OR  (fg100mSTimerCount3 > _100MS_EVENT_PERIOD))
				{
				// no need to check for overrun on 100mS events

				// set event flag
				BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK3);

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

			if ( (IS_BITSET(efMotionTimerEvents[MOTOR_AZIMUTH], EF_MTN_TIMER_25MS_TICK)) AND (pgeMotionPhase[MOTOR_AZIMUTH] IS_NOT PHASE_OPEN_LOOP) )
				{
				MotionFSM(MOTOR_AZIMUTH);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
				}

			#ifdef USE_ELEVATION
				// perhaps this should be moved to the next 25mS tick?
				if ( (IS_BITSET(efMotionTimerEvents[MOTOR_ELEVATION], EF_MTN_TIMER_25MS_TICK)) AND (pgeMotionPhase[MOTOR_ELEVATION] IS_NOT PHASE_OPEN_LOOP) )
					{
					MotionFSM(MOTOR_ELEVATION);			// event(s) to be processed, so run the Motion FSM without waiting for the timer
					}
			#endif	// USE_ELEVATION

			#ifdef USE_MOTION_FSM_TIMER_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
			#endif

			// clear calling event
			BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK4);

			continue;								// we have processed something, so restart at the top of the loop
													// this prevents processing EF_SCHED_25MS_TICK4 and EF_SCHED_100MS_TICK2 on the same 5mS tick
			}


		//-------------------------------
		//			25mS Tick 5
		//-------------------------------
		if (IS_BITSET(efSchedulerEvents, EF_SCHED_25MS_TICK5))
			{
			
			// bump 100mS tick counter
			fg100mSTimerCount4--;

			// check for timer complete - or (ACK!) rolled under
			if ((fg100mSTimerCount4 IS (unsigned int)0) OR  (fg100mSTimerCount4 > _100MS_EVENT_PERIOD))
				{
				// no need to check for overrun on 100mS events

				// set event flag
				BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK4);

				// reload timer
				fg100mSTimerCount4 = _100MS_EVENT_PERIOD;
				}


			// clear calling event
			BITCLEAR(efSchedulerEvents, EF_SCHED_25MS_TICK5);

			continue;									// we have processed something, so restart at the top of the loop
														// this prevents processing EF_SCHED_25MS_TICK5 and EF_SCHED_100MS_TICK3 on the same 5mS tick
			}


		//-----------------------------------------------------------------
		//		Handle 100mS Tick events
		//-----------------------------------------------------------------

		if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK1))
			{
			// run event
//			DebugLEDsTick();
                     // SendMBMessage(SERIAL_MENU_UART,xbee_addr,2, WAIT_FOR_DISPLAY);
                    //DisplayMessage(SERIAL_MENU_UART, "test", WAIT_FOR_DISPLAY);
			#ifdef USE_MOVE_SEQ_TICK_TRIGGER
				Trigger1Level(1);						// trigger to allow viewing this event on a scope
			#endif

			MoveSequenceTick();							// MoveSequenceFSM Timers()
                        if(dly10>10){
                        Calculate_Avg_ADC();
                        dly10=0;
                        }dly10++;
			#ifdef USE_MOVE_SEQ_TICK_TRIGGER
				Trigger1Level(0);						// trigger to allow viewing this event on a scope
			#endif


			#ifdef USE_BUTTON_PROC_FSM_TRIGGER
				Trigger1Level(1);						// trigger to allow viewing this event on a scope
			#endif

                        #ifdef SWITCH_INIT
                        
                        Switch_processing();
                        #endif

			ButtonProcessingFSM();

			#ifdef USE_BUTTON_PROC_FSM_TRIGGER
				Trigger1Level(0);						// trigger to allow viewing this event on a scope
			#endif

			#ifdef USE_HARD_STALL_RESET_TIMER
				// handle Hard Stall Reset Delay Timer
				if (IS_BITSET(efTimerEvents, EF_TIMER_HARD_STALL))
					{
					// Hard Stall Reset Delay timer is enabled

					// bump Hard Stall Reset Delay timer
					fgHardStallResetDelayTimerCount--;

					if ( fgHardStallResetDelayTimerCount IS 0 )
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

			continue;									// we have processed something, so restart at the top of the loop
			}


		if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK2))
			{
			// run event
//			AppLEDsTick();

			#ifdef USE_MOVE_SEQ_FSM_TRIGGER
				Trigger1Level(1);						// trigger to allow viewing this event on a scope
			#endif

//			if ((efMoveSequenceEvents IS_NOT NO_EVENTFLAGS) OR (IsMoveSequenceComplete() IS_FALSE))
//				MoveSequenceFSM();
                        if ((efMoveSequenceEvents IS_NOT NO_EVENTFLAGS) OR (IsMoveSequenceComplete() IS_FALSE) OR(IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW))OR(IS_BITSET(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW)))
				MoveSequenceFSM();
			#ifdef USE_MOVE_SEQ_FSM_TRIGGER
				Trigger1Level(0);						// trigger to allow viewing this event on a scope
			#endif

			// clear calling event flag
			BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK2);

			continue;									// we have processed something, so restart at the top of the loop
			}


		if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK3))
			{
			// run event
			#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
			#endif

			if ( ((efMotionPhaseCommands[MOTOR_AZIMUTH] IS_NOT NO_EVENTFLAGS) OR (IsCommandComplete(MOTOR_AZIMUTH) IS_FALSE))
				AND (pgeMotionPhase[MOTOR_AZIMUTH] IS_NOT PHASE_OPEN_LOOP) )
			{
				MotionPhaseFSM(MOTOR_AZIMUTH);
			}

			if ( ((efMotionPhaseCommands[MOTOR_ELEVATION] IS_NOT NO_EVENTFLAGS) OR (IsCommandComplete(MOTOR_ELEVATION) IS_FALSE))
				AND (pgeMotionPhase[MOTOR_ELEVATION] IS_NOT PHASE_OPEN_LOOP) )
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

			continue;				// we have processed something, so restart at the top of the loop
			}

                  if (IS_BITSET(efSchedulerEvents, EF_SCHED_100MS_TICK4))
			{
                        // clear calling event flag
			BITCLEAR(efSchedulerEvents, EF_SCHED_100MS_TICK4);

			// run event
			#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(1);				// trigger to allow viewing this event on a scope
			#endif

			#if defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_INCLINOMETER_FEEDBACK)
                        
                        // read accelerometer values
                        if (ReadInclinometerSample(&pgInclination) IS TRUE)							// read accelerometer, calculate 3D angles
                        {
                            fgbI2CErrorCtr = 0;
                            fgbI2CHrdStlCtr  = 0;
                            IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);			// add to averaging array, calculate new running average
                            //check beyond limits in both Manual and Auto for extreme cases
                            if(fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle) >= EXTREME_ANGLES)
                            {
                                ClearCommandStarted(MOTOR_AZIMUTH);
                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                #endif
                                Finish_MotionStats(MOTOR_AZIMUTH);
                                // ==>> does the previous command ever get completed?
                                SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                ResetMotionFSM();
                                continue;
                            }

                            // EF_MOTION_SENSOR_ACTIVE is set when the PWM is configured for FORWARD or REVERSE, before the start of motion
                            // EF_MOTION_SENSOR_ACTIVE is cleared when MotionFSM.c: MotionFSM() enters ST_MOTION_STOPPED for the first time
                            // so this will only execute DURING a move, and NOT when stopped.
                            if(IS_BITSET(efMotionSensorEvents[MOTOR_AZIMUTH], EF_MOTION_SENSOR_ACTIVE) IS_TRUE)
                            {
                                    if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
                                    {
                                        if(bTrackerDirection == 1)
                                        {
                                            if(((fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle-(ptrRTCC_RAM_AppParameters->fSetPoint_AZ))) <= MIN_FLUCT_ANGLE)
                                                    OR(pgAngleAverage.fX_Angle >= (ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - MIN_FLUCT_ANGLE)))
                                            {
                                                ClearCommandStarted(MOTOR_AZIMUTH);
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                                #endif
                                                Finish_MotionStats(MOTOR_AZIMUTH);
                                                // ==>> does the previous command ever get completed?
                                                SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
                                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                                ResetMotionFSM();
                                                continue;
                                            }
                                        }
                                        else if(bTrackerDirection == 2)
                                        {
                                            if(((fgdefferenceAngle = fabs(pgAngleAverage.fX_Angle-(ptrRTCC_RAM_AppParameters->fSetPoint_AZ))) <= MIN_FLUCT_ANGLE)
                                                    OR(pgAngleAverage.fX_Angle <= (ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse + MIN_FLUCT_ANGLE)))
                                            {
                                                ClearCommandStarted(MOTOR_AZIMUTH);
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                                #endif
                                                Finish_MotionStats(MOTOR_AZIMUTH);
                                                // ==>> does the previous command ever get completed?
                                                SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
                                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                                ResetMotionFSM();
                                                continue;
                                            }
                                        }
                                    }                                                                                                                // clear error counter; we are only interested in consecutive errors
                                    else
                                    {
                                        if(MAN_STOW == 1)
                                        {
                                           if((pgAngleAverage.fX_Angle <= (ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse + MIN_MANUAL_ANGLE))
                                              OR(pgAngleAverage.fX_Angle >= (ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - MIN_MANUAL_ANGLE))
                                              OR((int)pgAngleAverage.fX_Angle == 0))
                                            {
                                                MAN_STOW = 0;
                                                ClearCommandStarted(MOTOR_AZIMUTH);
                                                #if defined(USE_MOVE_SEQ_FSM_STEP_VERBOSE)
                                                DisplayMessage(SERIAL_MENU_UART,"Reached setpont", NO_WAIT_FOR_DISPLAY);
                                                #endif
                                                Finish_MotionStats(MOTOR_AZIMUTH);
                                                // ==>> does the previous command ever get completed?
                                                								// mark command as started so we cannot misinterpret completion
                                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                                ResetMotionFSM();
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
                                                DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                                #endif
                                                Finish_MotionStats(MOTOR_AZIMUTH);
                                               								// mark command as started so we cannot misinterpret completion
                                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                                ResetMotionFSM();
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
                                                DisplayMessage(SERIAL_MENU_UART,"\r\nReached setpont\r\n", WAIT_FOR_DISPLAY);
                                                #endif
                                                Finish_MotionStats(MOTOR_AZIMUTH);
                                                BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                                                ResetMotionFSM();
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
                                if (fgbI2CErrorCtr IS MAX_I2C_ERROR_CNT)								// if too many consecutive errors, stop motor (see inclinometer.h)
                                {
                                    fgbI2CErrorCtr = 0;
                                    ClearCommandStarted(MOTOR_AZIMUTH);
                                    MAN_EAST = MAN_WEST = 0;
                                    Finish_MotionStats(MOTOR_AZIMUTH);
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
                                     if(fgbI2CHrdStlCtr IS_NOT MAX_I2C_HrdStal_CNT)
                                     {
                                          // inclinometer read error/failure
                                        ++fgbI2CErrorCtr;
                                        I2CReset();
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
                        
			#endif	//  defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_INCLINOMETER_FEEDBACK)

			#ifdef USE_MOTION_PHASE_FSM_TRIGGER
				Trigger1Level(0);				// trigger to allow viewing this event on a scope
			#endif

			

			continue;				// we have processed something, so restart at the top of the loop
			}

		#ifdef USE_PGBTERMINATE
			// this is an incomplete effort at providing a software restart..
			if (pgbTerminate IS_TRUE)
				{
				break;					// exit while loop, exit main(), restart
				}
		#endif
   		}		// end for(;;)


}


void __ISR(_CORE_TIMER_VECTOR, ipl2) CoreTimerHandler(void)
{
    // .. things to do

	// ***********************************************
	//				5mS Tick
	// ***********************************************
	// 5mS tick is generated from Core Timer
	#ifdef USE_5MS_INT_TRIGGER
		Trigger1Toggle();
	#endif

	// check for 5mS event overrun
	if (IS_BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT))
		{
		RuntimeError(TIMER_ERROR_5MS_TICK_OVERRUN);
		}

	#ifndef _lint		// too many complex lint errors here!
		// reload the counter (update the period)
		UpdateCoreTimer(CORE_TICK_RATE);
	#endif	// _lint

	// set event flag
	BITSET(efTimerEvents, EF_TIMER_5MS_TICK_INT);

	#ifndef _lint		// too many complex lint errors here!
		// clear the calling interrupt flag
		mCTClearIntFlag();
	#endif	// _lint

}
