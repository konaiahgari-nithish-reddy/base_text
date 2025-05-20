// *************************************************************************************************
//									M o t i o n S e n s o r . c
// *************************************************************************************************
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Sensor measure, read functions, for 2 axis
//
// *************************************************************************************************

// some of this code is derived from the Microchip example code incap_capture_event.c
// which, like most Microchip demo code, was full of errors

// this #define is used to control access to file global data, which is shared at the source code level
// between MotionSensor.c and Inclinometer.c

#define MOTIONSENSOR_C

//-------------------------------------------------------------------------------------------------------
//	Include Files
//-------------------------------------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//#include <math.h>				// M_PI
//#include <string.h>				// Microchip string functions
// see hlpC18.chm online help for so-called documentation
//#include <ctype.h>				// tolower()

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
//#include "HardwareProfile.h"
#include "EventFlags.h"			// event flag definitions and globals

//#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
//#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
//#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
//#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM

//#include "ADCRead.h"			// adc access functions
//#include "AppTimer.h"			// timer errors

//#include "TimeDelay.h"			// delay in increments of 10uS

//#include "Stubs.h"
#include "CoordTranslate.h"
#ifdef USE_MMA8452Q_INCLINOMETER
//	#include "mma845x.h"              // MMA845xQ definitions
#include "Inclinometer.h"
#endif

#ifdef DEFINE_GLOBALS
#error "DEFINE_GLOBALS not expected here"
#endif

#ifdef USE_HALL_SENSOR_FEEDBACK

//-------------------------------------------------------------------------------------------------------
// File Global Variables
//-------------------------------------------------------------------------------------------------------

#ifdef USE_ELEVATION_LINEAR_DRIVE_SIMULATOR
// initialize counter for approx ratio of MSI ticks, slew drive to linear actuator drive
//	PRIVATE_INIT UINT8 fgElevationLinearDriveSimulatorCtr = EL_LINEAR_DRIVE_SIM_TICK_COUNT;
UINT16 fgElevationLinearDriveSimulatorCtr = EL_LINEAR_DRIVE_SIM_TICK_COUNT;
#endif	// USE_ELEVATION_LINEAR_DRIVE_SIMULATOR

//-------------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------------

#ifdef USE_HALL_EFFECT_FEEDBACK
void MotionSensorInterruptHandler(enum tagMotors eMotor);
#endif

// *****************************************************************************
//					M o t i o n S e n s o r _ I n i t ( )
// *****************************************************************************

void MotionSensor_Init(void)
{

	// ********************************
	//		Setup Feedback Simulator
	// ********************************
	// if the feedback simulator is being used (instead of the actual Hall sensors),
	///initialize the pins (F0, F1) that will drive the motion sensor inputs
#ifdef	USE_FEEDBACK_SIMULATOR
	InitMotionFeedbackSimulatorPins();
#endif		// USE_FEEDBACK_SIMULATOR

	// ********************************
	//		Enable Interrupts
	// ********************************

#ifndef _lint		// too many complex PC-Lint errors in hardware access
	// clear any pending interrupt requests
	AZ_ICClearIntFlag();
	EL_ICClearIntFlag();

	// make sure interrupts are OFF
	MotionSensor_DisableInt(AXIS_AZIMUTH);
	MotionSensor_DisableInt(AXIS_ELEVATION);

	// Setup Timer 3
	//	Timer 3 ON
	//	Divide by 256 (note timer input is 40MHZ; sample code was 10MHZ)
	//  Internal source
	// This should provide a tick of 0.5mS. The shortest Input Capture cycle we are likely to measure is 15mS

	//		   <<-------------T3CON------------->>   PR3	TMR3 = 0
	OpenTimer3(T3_ON | T3_PS_1_256 | T3_SOURCE_INT, 0xFFFF);
#endif	// _lint

	// Set I/O pin direction
	// from PIC32 Family Reference Manual Section 15. Input Capture page 15-18:
	// When the Input Capture module is enabled, the user application must ensure that the I/O pin
	//		direction is configured for an input by setting the associated TRIS bit. The pin direction is not set
	//		when the Input Capture module is enabled. Furthermore, all other peripherals multiplexed with
	//		the input pin must be disabled.
	// PIC32 SK hardware:
	//		IC1: RD8	Azimuth
	//		IC2: RD9	Elevation
	// SmartTrak hardware:
	//		IC1: RD8	Elevation
	//		IC3: RD10	Azimuth

	InitMotionSensorPins();

#ifndef _lint		// too many complex PC-Lint errors in hardware access
	// Enable Input Capture Modules 1,2
	// - IC_EVERY_RISE_EDGE		Capture Every RISING edge
	// - IC_CAP_16BIT			16 bit counter
	// - IC_INT_1CAPTURE		Enable capture interrupts on each capture
	// - IC_TIMER3_SRC			Use Timer 3 source
	// - IC_FEDGE_RISE			Capture rising edge first
	// - IC_IDLE_STOP			IC stop in sleep mode (debug?)
	// - IC_ON					input capture ON
	AZ_OpenCapture(IC_EVERY_RISE_EDGE | IC_CAP_16BIT | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON);
	EL_OpenCapture(IC_EVERY_RISE_EDGE | IC_CAP_16BIT | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON);
#endif	// _lint

	// Input Capture interrupts are enabled when PWM configuration changes and starts motion

	// initialze current Orientation from values stored in NV RAM

#ifdef USE_ELEVATION_LINEAR_DRIVE_SIMULATOR
	// initialize counter for approx ratio of MSI ticks, slew drive to linear actuator drive
	fgElevationLinearDriveSimulatorCtr = EL_LINEAR_DRIVE_SIM_TICK_COUNT;
#endif	// USE_ELEVATION_LINEAR_DRIVE_SIMULATOR

}

// *****************************************************************************
//					M o t i o n S e n s o r _ T i c k ( )
// *****************************************************************************
// called from the foreground loop to process the current speed and generate PWM change flags

void MotionSensor_Tick(enum tagMotors eMotor)
{

#ifdef USE_MOTION_SENSOR_TICK_TRIGGER
	Trigger1Level(1);					// trigger to allow viewing this event on a scope
#endif

	// make sure the expected calling flag is in fact SET
	if (IS_BITCLEAR(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK))
	{
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_TICK);
	}

	// clear the calling event flag
	BITCLEAR(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK);

	// save the Current Orientation to MCU RAM copy of RTCC NV RAM, so we can recover it if power is lost
	ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth = CurrentPosition_Read(MOTOR_AZIMUTH);
	ptrRTCC_RAM_MechanicalOrientation->lLastElevation = CurrentPosition_Read(MOTOR_ELEVATION);

#ifdef USE_MOTION_SENSOR_TICK_RAM_UPDATE_TRIGGER
	Trigger1Level(1);					// trigger to allow viewing this event on a scope
#endif

#ifdef USE_DS3232_RTCC
	// write MCU RAM copy of Current Orientation to NV RTCC RAM, so we can recover it if power is lost
	IGNORE_RETURN_VALUE UpdateRTCCRAMOrientation();
#endif

#ifdef USE_MOTION_SENSOR_TICK_RAM_UPDATE_TRIGGER
	Trigger1Level(0);					// trigger to allow viewing this event on a scope
#endif

	// speed handling depends on the motion phase
	switch(pgeMotionPhase[eMotor])
	{
	case PHASE_STOPPED:					// motion has actually STOPPED (Motion sensor tick timeout), includes STALLED
		RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
		break;

	case PHASE_ACCELERATION:			// accelerating speed
		switch(pgeMotionType[eMotor])
		{
		case MOTION_STARTING:
			// this is the FIRST MSI tick after starting up, so we cannot do any calculations yet
			// pgMotionStats.wAccelerationPWMAdjustmentCount = 0;

			// in acceleration, every MSI tick results in a PWM value update
			BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);

			// we have now gone through ONE MSI interrupt and ONE MSI Tick routine, so on the NEXT MSI Tick, we can process the measured speed
			pgeMotionType[eMotor] = MOTION_POWERED;
			break;

		case MOTION_POWERED:
			// we have been in motion, so we can calculate speed
			// we are NOW past the very first MSI tick, for which we do NOT have any speed info

			// bounds check incremented MotionProfile speed index
			if (pgbMotionProfileSpeedIndex[eMotor] <= (pgbMotionProfileIndexMax[eMotor] - pgbMotionProfileIndexIncrement[eMotor]))
			{
				// bump index into motion profile table of expected speed values
				// calculate new motion profile index, and look at expected speed value (in Tcy counts)

				// bump motion profile table index
				pgbMotionProfileSpeedIndex[eMotor] += pgbMotionProfileIndexIncrement[eMotor];

				// read expected speed from motion profile table
				fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

				// keep track of speed error for status display
				pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);

#ifdef MOTION_ERROR_TABLE
				// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
				// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
				if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
				{
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = pgbPWMDutyCycle[eMotor];

					++pgwMotionProfileSpeedErrorIndex;					// bump table index
				}
#endif

				// compare current speed to expected maximum speed.
				// Speed is measured as a count of Tcy ticks, between MSI ticks, so smaller is faster
				if (pguCurrentSpeed[eMotor] < (fguExpectedSpeed[eMotor] - MTN_SENSOR_SPEED_TOLERANCE))		// smaller is faster
				{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);
				}
				else if (pguCurrentSpeed[eMotor] > (fguExpectedSpeed[eMotor] + MTN_SENSOR_SPEED_TOLERANCE))
				{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
				}
				else
				{
					// in acceleration, every MSI tick results in a PWM value update of SOME type
					BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
				}
			}
			else
			{
				RuntimeError(MTN_SENSOR_ERROR_NO_EVENT_GENERATED);
			}

			break;

			// NONE of these motion types should occur during PHASE_ACCELERATION
		case MOTION_INIT:				// initial state, only at power up
		case MOTION_COASTING:			// motor is OFF, some coasting MAY occur
		case MOTION_BRAKING:			// motor is OFF, brake is on, some coasting MAY occur
		case MOTION_STOPPED:			// motion has actually STOPPED (Motion Sensor tick timeout)
		case MOTION_STALLED:			// system has STALLED; no exit
		default:
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
			break;

		}		// end 	switch(pgeMotionType[eMotor])
		break;
		case PHASE_OPEN_LOOP:				// used ONLY for open loop operation
			pgeMotionType[eMotor] = MOTION_POWERED;			// to allow speed measurement
			// in Open Loop mode, nothing else to do here; NO speed adjustments
			break;

		case PHASE_CONSTANT_SPEED:			// constant speed operation
			if (pgeMotionType[eMotor] IS MOTION_POWERED)
			{
				// NOTE that index into motion profile table of expected speed values does NOT change during the Constant Speed phase
				// calculate motion profile index (which does not change during Constant Speed), and look up expected speed value (in Tcy counts)
				fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

				// keep track of speed error for status display
				pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);

#ifdef MOTION_ERROR_TABLE
				// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
				// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
				if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
				{
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
					pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = pgbPWMDutyCycle[eMotor];

					++pgwMotionProfileSpeedErrorIndex;					// bump table index
				}
#endif


				// compare current speed to expected maximum speed. 
				// Speed is measured as a count of Tcy ticks, between MSI ticks, so smaller is faster
				if (pguCurrentSpeed[eMotor] < (fguExpectedSpeed[eMotor] - MTN_SENSOR_SPEED_TOLERANCE))		// smaller is faster
				{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_FAST);
				}
				else if (pguCurrentSpeed[eMotor] > (fguExpectedSpeed[eMotor] + MTN_SENSOR_SPEED_TOLERANCE))
				{
					// set event flag for motion FSM
					BITSET(efMotionEvents[eMotor], EF_MOTION_TOO_SLOW);
				}
			}
			else
			{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
			}
			break;

		case PHASE_DECELERATION:			// decelerating
			// pgMotionStats.wQDecelerationPWMAdjustmentCount = 0;
			// NOTE: during deceleration, Motion Profile index pgbMotionProfileSpeedIndex is adjusted by MotionFSM.c (why?)
			// we are no longer adjusting speed, so there is no error to measure
			fguExpectedSpeed[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + MSI_SPEED_OFFSET];

			// keep track of speed error for status display ONLY
			pgsSpeedError[eMotor] = (MOTION_PROFILE_SPEED_ERR_TYPE)(fguExpectedSpeed[eMotor] - pguCurrentSpeed[eMotor]);


#ifdef MOTION_ERROR_TABLE
			// a positive value means that the current speed is numerically less than the expected speed - meaning we are going TOO FAST
			// a negative value means that the current speed is numerically more than the expected speec - meaning we are going TOO SLOW
			if (pgwMotionProfileSpeedErrorIndex < SPEED_ERROR_TABLE_LEN)
			{
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bMotionProfileIndex = pgbMotionProfileSpeedIndex[eMotor];
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].ExpectedSpeed = fguExpectedSpeed[eMotor];
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].MeasuredSpeed = pguCurrentSpeed[eMotor];
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].SpeedError = pgsSpeedError[eMotor];
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].cPWMCorrection = pgcDutyCycleCorrection[eMotor];
				pgMotionProfileSpeedErrorTbl[pgwMotionProfileSpeedErrorIndex].bPWM = pgbPWMDutyCycle[eMotor];

				++pgwMotionProfileSpeedErrorIndex;					// bump table index
			}
#endif

			if (pgeMotionType[eMotor] IS MOTION_POWERED)
			{
				// in deceleration, every MSI tick results in a PWM value update
				BITSET(efMotionEvents[eMotor], EF_MOTION_PWM_UPDATE);
			}
			else
			{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
			}
			break;

		case PHASE_MINIMUM_PWM:				// constant speed at minimum PMW
			// pgMotionStats.wQDecelerationPWMAdjustmentCount = 0;
			if (pgeMotionType[eMotor] IS MOTION_POWERED)
			{
				;
			}
			else
			{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
			}
			break;

		case PHASE_COASTING:				// motor is OFF, some coasting MAY occur
			if ((pgeMotionType[eMotor] IS MOTION_COASTING) OR (pgeMotionType[eMotor] IS MOTION_BRAKING))
			{
				;
			}
			else
			{
				// not a valid MOTION_xxx type
				RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_TICK);
			}
			break;

		case PHASE_INIT:					// not even valid here
		default:
			RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
			break;
	}

	// make sure the expected calling flag is STILL CLEAR
	if (IS_BITSET(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK))
	{
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_TICK);
	}

	// NOTE: we do not make a general check for unprocessed efADCEvents events here, because there is only ONE event type
	// (EF_MTN_SENSOR_MOTION_SENSOR_TICK) and it is interrupt generated - so another event may occur at ANY time.

#ifdef USE_MOTION_SENSOR_TICK_TRIGGER
	Trigger1Level(0);					// trigger to allow viewing this event on a scope
#endif

}



//---------------------------------------------------------------------
// Read current state of Motion Sensor Interface Pins
// this is implemented as a hardware-specific input, generalized output routine 

// used ONLY for serial menus MenuFSM.c

// D8	PIN_ELEVATION_SENSOR
// D10	PIN_AZIMUTH_SENSOR

UINT16 GetMotionSensorState( void )
{

	//	BYTE	cSensorState = 0;

	return ReadMotionSensorPins();

}



// *****************************************************************************
//				M o t i o n S e n s o r _ x x x x x I n t ( )
// *****************************************************************************

// enable or disable Motion Sensor interrupts
// we need to turn off the interrupts AFTER a motion stall, so that we do not get any additional interrupts until intentional motion starts again

// Enable Encoder interrupts
// Called from MotionFSM() upon entering ST_MOTION_ACCELERATE and during ST_MOTION_SOFT_STALL
void MotionSensor_EnableInt(enum tagAxis eAxis)
{
	WORD wDummy;

	switch(eAxis)
	{
#ifndef _lint		// too many complex PC-Lint errors in hardware access
	case AXIS_AZIMUTH:
		// clear any pending interrupt requests
		AZ_ICClearIntFlag();

		while (AZ_ICCONbits.ICBNE)		// flush FIFO by reading all available data
			wDummy = AZ_ICBUF;			// reads the input capture buffer

		// Setup Input Capture 1 Interrupts
		// IC_INT_ON			Enable input capture interrupt  IC_INT_ON
		// IC_INT_PRIOR_3		Set priority for level 3
		// IC_INT_SUB_PRIOR_0	Set sub priority for level 0

		AZ_ConfigIntCapture(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_0);

		// enabling interrupts clears Input Capture FIFO
		break;

	case AXIS_ELEVATION:
		// clear any pending interrupt requests
		EL_ICClearIntFlag();

		while (EL_ICCONbits.ICBNE)		// flush FIFO by reading all available data
			wDummy = EL_ICBUF;			// reads the input capture buffer

		// Setup Input Capture 1 Interrupts
		// IC_INT_ON			Enable input capture interrupt  IC_INT_ON
		// IC_INT_PRIOR_3		Set priority for level 3
		// IC_INT_SUB_PRIOR_0	Set sub priority for level 0

		EL_ConfigIntCapture(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_1);

		// enabling interrupts clears Input Capture FIFO
		break;
#endif	// _lint

	default:
		RuntimeError(MTN_SENSOR_ERROR_INVALID_AXIS);
		break;
	}

}

// Disable Encoder interrupts
// Called from MotionFSM() upon entering ST_MOTION_STOPPED
void MotionSensor_DisableInt(enum tagAxis eAxis)
{

	WORD wDummy;

	switch(eAxis)
	{
#ifndef _lint		// too many complex PC-Lint errors in hardware access
	case AXIS_AZIMUTH:
		// clear any pending interrupt requests
		AZ_ICClearIntFlag();

		// Disable Input Capture 1 interrupts
		// IC_INT_OFF			Enable input capture interrupt  IC_INT_ON
		AZ_ConfigIntCapture(IC_INT_OFF);

		// discard any remaining Motion Capture (Input Capture) FIFO data
		while (AZ_ICCONbits.ICBNE)		// if data is available
			wDummy = AZ_ICBUF;			// reads the input capture buffer

		break;

	case AXIS_ELEVATION:
		// clear any pending interrupt requests
		EL_ICClearIntFlag();

		// Disable Input Capture 2 interrupts
		// IC_INT_OFF			Enable input capture interrupt  IC_INT_ON
		EL_ConfigIntCapture(IC_INT_OFF);

		// discard any remaining Motion Capture (Input Capture) FIFO data
		while (EL_ICCONbits.ICBNE)		// if data is available
			wDummy = EL_ICBUF;			// reads the input capture buffer

		break;
#endif		// _lint

	default:
		RuntimeError(MTN_SENSOR_ERROR_INVALID_AXIS);
		break;
	}


}

// *****************************************************************************
//			M o t i o n S e n s o r I n t e r r u p t H a n d l e r( )
// *****************************************************************************

// this is a very low speed interrupt handler (less than 100 Hz)
// it is currently called from the high priority interrupt handler (see Interrupt.c)
// Perhaps it could just be moved to the foreground loop?

// PIC32 SK hardware:
//		IC1: RD8	Azimuth
//		IC2: RD9	Elevation
// SmartTrak hardware:
//		IC3: RD10	Azimuth
//		IC1: RD8	Elevation

#ifndef _lint		// too many complex PC-Lint errors in hardware access

#if defined (PLATFORM_PIC32_SK)
void __ISR( _INPUT_CAPTURE_1_VECTOR, ipl3) Capture1(void)
				{
	MotionSensorInterruptHandler(MOTOR_AZIMUTH);

	// clear interrupt flag to allow next Input Capture interrupt
	AZ_ICClearIntFlag();
				}

void __ISR( _INPUT_CAPTURE_2_VECTOR, ipl3) Capture2(void)
				{
	MotionSensorInterruptHandler(MOTOR_ELEVATION);

	// clear interrupt flag to allow next Input Capture interrupt
	EL_ICClearIntFlag();
				}
#elif defined (PLATFORM_SMARTTRAK_V1)
void __ISR( _INPUT_CAPTURE_3_VECTOR, ipl3) Capture3(void)
				{
	MotionSensorInterruptHandler(MOTOR_AZIMUTH);

	// clear interrupt flag to allow next Input Capture interrupt
	AZ_ICClearIntFlag();
				}

void __ISR( _INPUT_CAPTURE_1_VECTOR, ipl3) Capture1(void)
				{
	MotionSensorInterruptHandler(MOTOR_ELEVATION);

	// clear interrupt flag to allow next Input Capture interrupt
	EL_ICClearIntFlag();
				}
#endif		// PLATFORM_SMARTTRAK_V1

#endif	// _lint


// Called from above interrupt handlers
// In Feedback Simulator mode, called from main() Scheduler when SimulatorTickCtr expires

void MotionSensorInterruptHandler(enum tagMotors eMotor)
{

	WORD wCurrentTimerValue[NUM_MOTORS];

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
	Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif


	// ********************************************
	//		Check state of Motion Sensor Input
	// ********************************************
	// the Hall Effect Motion Sensor input should still be HI here
	Delay10us((UINT32)(MOTION_SENSOR_EDGE_DELAY_US / 10));			// delay before even looking at motion sensor signal, to eliminate processing noise spikes as input edges

	// if sensor pin is LOW, this interrupt is in response to a short noise pulse, and should be ignored.
	if ((eMotor IS MOTOR_AZIMUTH) AND ((ReadMotionSensorPins() & PIN_AZIMUTH_SENSOR) IS_NOT PIN_AZIMUTH_SENSOR))
	{
		WORD wDummy;

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
		Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

		// Hall Effect Sensor Input has already gone low
		RuntimeError(MTN_SENSOR_ERROR_INVALID_INPUT_LEVEL);

#ifndef _lint		// too many complex PC-Lint errors in hardware access

		// clear data from Input Capture FIFO so it is not read later
		if (AZ_ICCONbits.ICBNE)				// check for more data available
		{
			// clear any data from FIFO
			while (AZ_ICCONbits.ICBNE)		// clear overrun by reading all available data
				wDummy = AZ_ICBUF;			/* reads the input capture buffer */
		}

		if (AZ_ICCONbits.ICOV)				// check for FIFO overflow
		{
			RuntimeError(TIMER_ERROR_MTN_SENSOR_FIFO_OVERRUN);
			while (AZ_ICCONbits.ICBNE)		// clear overrun by reading all available data
				wDummy = AZ_ICBUF;			/* reads the input capture buffer */
		}

#endif	// _lint

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
		Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

		return;								// nothing else to do; caller clears interrupt flag

	}

	// handler as above for MOTOR_ELEVATION
	// if sensor pin is LOW, this interrupt is in response to a short noise pulse, and should be ignored.
	if ((eMotor IS MOTOR_ELEVATION) AND ((ReadMotionSensorPins() & PIN_ELEVATION_SENSOR) IS_NOT PIN_ELEVATION_SENSOR))
	{
		WORD wDummy;

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
		Trigger1Level(1);				// trigger to allow viewing this event on a scope
#endif

		// Hall Effect Sensor Input has already gone low
		RuntimeError(MTN_SENSOR_ERROR_INVALID_INPUT_LEVEL);

#ifndef _lint		// too many complex PC-Lint errors in hardware access
		// clear data from Input Capture FIFO so it is not read later
		if (EL_ICCONbits.ICBNE)				// check for more data available
		{
			// clear any data from FIFO
			while (EL_ICCONbits.ICBNE)		// clear overrun by reading all available data
				wDummy = EL_ICBUF;			/* reads the input capture buffer */
		}

		if (EL_ICCONbits.ICOV)				// check for FIFO overflow
		{
			RuntimeError(TIMER_ERROR_MTN_SENSOR_FIFO_OVERRUN);
			while (EL_ICCONbits.ICBNE)		// clear overrun by reading all available data
				wDummy = EL_ICBUF;			/* reads the input capture buffer */
		}
#endif	// _lint

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
		Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif

		return;								// nothing else to do; caller clears interrupt flag

	}

#ifdef USE_ELEVATION_LINEAR_DRIVE_SIMULATOR
	if (eMotor IS MOTOR_ELEVATION)
	{
		// when using a slew drive to simulate the elevation linear actuator, the MSI ticks are WAY too fast
		// based on current spreadsheet values, we want to use only 1 in 13 MSI ticks
		//		calculation: slew drive: 2 ticks per motor revolution, linear drive: 0.1586 ticks per motor revolution
		--fgElevationLinearDriveSimulatorCtr;
		if (fgElevationLinearDriveSimulatorCtr > 0)
		{
			return;				// have not yet skipped full count of ticks
		}
		else
		{
			// restart counter
			fgElevationLinearDriveSimulatorCtr = EL_LINEAR_DRIVE_SIM_TICK_COUNT;
		}
	}
#endif	// USE_ELEVATION_LINEAR_DRIVE_SIMULATOR



	// ********************************************
	//		track overall MSI Ticks
	// ********************************************
	// bump MSI ticks-in-current-motion-phase and total-ticks-in-move counters
	// (this ABSOLUTELY needs to be in the ISR)
	++pgwStateMSICtr[eMotor];							// increment count of MSI ticks in current motion state; always checked for >= LIMIT
	++pgulMoveTotalMSICtr[eMotor];						// count total MSI ticks in move

	// test for completed MSI ticks in motion phase
	// (this should be in the ISR to avoid latency issues)
	if (pgwStateMSICtr[eMotor] >= pgwStateMSILimit[eMotor])				// if we have completed the MSI ticks for the motion phase
	{
		// check for flag overrun, which could occur if we are coasting to a stop
		if (IS_BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE))
		{
			RuntimeError(MTN_SENSOR_ERROR_MSI_COUNT_DONE_OVERRUN);
		}
		BITSET(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);	// set event flag for faster processing
	}

	// ********************************************
	//		Read Input Capture Value from Timer
	// ********************************************
	// read the Input Capture value, the time between Motion Sensor Ticks
	// (this ABSOLUTELY needs to be in the ISR)
	wCurrentTimerValue[eMotor] = ReadMSITimerInputCapture(eMotor);

	// ********************************************
	//		Track Position by counting ticks
	// ********************************************
	// keep track of the current position by bumping fglCurrentPosition on each Motion Sensor tick.
	// NOTE: the 0 point is arbitrary;this will require a 'move to center' capability to set the 0 point

	switch(pgePWMDirection[eMotor])
	{
	case PWM_DIR_REVERSE:
		--fglCurrentPosition[eMotor];
		break;

	case PWM_DIR_FORWARD:
		++fglCurrentPosition[eMotor];
		break;

	case PWM_DIR_STOPPED:
	case PWM_DIR_UNKNOWN:							// error value
	default:
		// if we do not have a direction as above, we should not get a motion sensor tick!
		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_INT);
		break;
	}


	// ********************************************
	//		track per-motion-phase MSI Ticks
	// ********************************************
	// keep track of Motion Sensor ticks per motion phase
	// this is done for debug logging, and is NOT required for the final system
	// (this should be in the ISR to avoid latency issues)
	switch(pgeMotionPhase[eMotor])
	{
	case PHASE_STOPPED:						// motion has actually STOPPED (Motion Sensor tick timeout), includes STALLED
		// this is a bit of a kludge, because we CERTAINLY do not exit the STOPPED state based on a state MSI tick count!
		BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag

		RuntimeError(MTN_SENSOR_ERROR_UNEXPECTED_EVENT);
		break;

	case PHASE_ACCELERATION:				// accelerating speed
		++pgwMSI_AccelerationCtr[eMotor];	// MSI ticks for each of the motion phases
		break;

	case PHASE_CONSTANT_SPEED:			// constant speed operation
	case PHASE_OPEN_LOOP:				// used ONLY for open loop operation
		++pgulMSI_ConstantSpeedCtr[eMotor];
		break;

	case PHASE_DECELERATION:			// decelerating
		++pgwMSI_DecelerationCtr[eMotor];
		break;

	case PHASE_MINIMUM_PWM:				// constant speed at minimum PMW
		++pgwMSI_MinimumPWMCtr[eMotor];

		// this is a bit of a kludge, because we do not exit the MINIMUM_PWM state based on a state MSI tick count
		// (we don't really know how many ticks will occur in this state; it is just whatever did not get done during deceleration.)
		//////			BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag
		break;

	case PHASE_COASTING:				// motor is OFF, some coasting MAY occur
		++pgwMSI_CoastCtr[eMotor];

		// this is a bit of a kludge, because we do not exit the COASTING state based on a state MSI tick count
		BITCLEAR(efMotionEvents[eMotor], EF_MOTION_MSI_COUNT_DONE);		// unconditionally clear flag
		break;

	case PHASE_INIT:					// not even valid here
	default:
		RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_PHASE);
		break;
	}



	// ********************************************
	//			Calculate Speed
	// ********************************************
	// (this does not involve any more measurements, so it does not HAVE to be in the ISR)

	switch(pgeMotionType[eMotor])
	{
	case MOTION_STARTING:
		// this is the FIRST MSI tick after starting up, so we cannot do any speed calculations yet
		// fgwLastTimerValue is set below

		// this must be the first encoder interrupt after being STOPPED, so we cannot calculate speed
		pguCurrentSpeed[eMotor] = 0;

		// update stall counter to 1.5 times (?) longest expected interval to avoid a motion stall during startup
		SetMotionStallCtr(eMotor, MTN_SENSOR_STARTUP_STALL_PERIOD(eMotor));

#ifdef OPEN_LOOP_MOVES_ONLY
		// kludge for open loop ONLY operation, which #ifdefs out most of the tick handler
		pgeMotionType[eMotor] = MOTION_POWERED;
#endif
		break;

	case MOTION_POWERED:
		// we have been in motion, so we can calculate speed

#ifdef CALC_AVERAGE_SPEED
		// keep the previous speed value
		fgwLastSpeed[eMotor] = pguCurrentSpeed[eMotor];
#endif

		// *********************************
		//	Calculate operating speed
		// *********************************
		// update the speed value, the interrupt-to-interrupt time, (in 6.4uS ticks)
		// check for (free running) counter rollover during the timed interval
		// NOTE: this is depedent on the free running T3 timer value stored during the PREVIOUS interrupt, fgwLastTimerValue[eMotor]
		if (wCurrentTimerValue[eMotor] > fgwLastTimerValue[eMotor])
		{
			// no counter rollover
			// calculate speed (number of counter ticks) since last interrupt, counter INCREMENTS
			pguCurrentSpeed[eMotor] = wCurrentTimerValue[eMotor] - fgwLastTimerValue[eMotor];
		}
		else
		{
			// counter has rolled over
			pguCurrentSpeed[eMotor] = wCurrentTimerValue[eMotor] + (MTN_SENSOR_TMR_MAX_VALUE - fgwLastTimerValue[eMotor]);
		}

#ifdef USE_ELEVATION_LINEAR_DRIVE_SIMULATOR
		if (eMotor IS MOTOR_ELEVATION)
		{
			// if we are trying to simulate the elevation linear actuator with a slew drive, only 1 of EL_LINEAR_DRIVE_SIM_TICK_COUNT
			// MSI ticks are processed. We also need to multiply the "speed" (tick count value by EL_LINEAR_DRIVE_SIM_TICK_COUNT
			//					pguCurrentSpeed[eMotor] *= EL_LINEAR_DRIVE_SIM_TICK_COUNT;
		}
#endif	// USE_ELEVATION_LINEAR_DRIVE_SIMULATOR

		// *********************************
		//		track speed values
		// *********************************
		// keep track of maximum speed - NOTE: speed is measured as a count of Tcy, so smaller is faster
		if  (pguCurrentSpeed[eMotor] < pgMotionStats[eMotor].MaximumSpeed)
		{
			pgMotionStats[eMotor].MaximumSpeed = pguCurrentSpeed[eMotor];
		}

		// keep track of minimum speed
		if  (pguCurrentSpeed[eMotor] > pgMotionStats[eMotor].MinimumSpeed)
		{
			pgMotionStats[eMotor].MinimumSpeed = pguCurrentSpeed[eMotor];
		}

		// update stall counter to 1.5 times (?) the current speed value
		// bounds check current speed value, calculation is (x) * 3 / 2 or (x) * 3
		if (pguCurrentSpeed[eMotor] < MAX_MTN_SENSOR_TICK_FOR_STALL )
		{
			SetMotionStallCtr(eMotor, MTN_SENSOR_MOTION_STALL_PERIOD(pguCurrentSpeed[eMotor]));
		}
		else
		{
			// using the current speed value will result in an undersize stall counter value
			// so limit counter value to maximum usable value
			SetMotionStallCtr(eMotor, MAX_MTN_SENSOR_TICK_FOR_STALL);
		}

#ifdef CALC_AVERAGE_SPEED
		// calculate the average speed
		++fgwSampleCtr;
		if (fgwSampleCtr > 100)
		{
			// restart all sampling
			fgwSampleCtr = 1;
			fgwSumOfSpeeds = 0;
			fgwLastSpeed = pguCurrentSpeed;		// to prevent meaningless Error From Sample values
		}

		fgwSumOfSpeeds += pguCurrentSpeed;
		pgwAverageSpeed = fgwSumOfSpeeds / fgwSampleCtr;		// note that sampleCtr is at LEAST 1

		// calculate error from average speed
		pgnErrorFromAverage = (int)pgwAverageSpeed - (int)pguCurrentSpeed;

		// calculate error from most recent sample
		pgnErrorFromSample = (int)fgwLastSpeed - (int)pguCurrentSpeed;
#endif

		break;

	case MOTION_COASTING:			// motor is OFF, some coasting MAY occur
	case MOTION_BRAKING:
		// power is off, and we are COASTING, this is processed just see how long we coast!

		// this must be the first encoder interrupt after being STOPPED, so we cannot calculate speed
		//			pguCurrentSpeed = 0;

		// update stall counter to 1.5 (?) times longest expected interval
		SetMotionStallCtr(eMotor, MTN_SENSOR_COASTING_STALL_PERIOD(eMotor));

		////RuntimeError(MTN_SENSOR_ERROR_COASTING_INT);		// this is an event, not an error!
		break;

	case MOTION_INIT:				// initial state, only at power up
	case MOTION_STOPPED:			// motion has actually STOPPED (Motion Sensor tick timeout)
	case MOTION_STALLED:			// system has STALLED; no exit
	default:
		RuntimeError(MTN_SENSOR_ERROR_INVALID_MOTION_TYPE_INT);
		break;
	}		// end 	switch(pgeMotionType[eMotor])

	// update stored timer value for next interrupt speed calculation
	fgwLastTimerValue[eMotor] = wCurrentTimerValue[eMotor];



	// ********************************************
	//		foreground processing flag
	// ********************************************
	// check for MSI tick overrun (if the flag is still set from the previous interrupt, the foreground loop did not process the flag in time)
	// (this ABSOLUTELY needs to be in the ISR)
	if (IS_BITSET(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK))
	{
		RuntimeError(MTN_SENSOR_ERROR_FOREGROUND_TICK_OVERRUN);
	}

	// set event flag for foreground loop processing,  MotionSensor_Tick()
	BITSET(efMotionSensorEvents[eMotor], EF_MOTION_SENSOR_TICK);

#ifdef USE_MOTION_SENSOR_INT_TRIGGER
	Trigger1Level(0);				// trigger to allow viewing this event on a scope
#endif
}

#endif	// USE_HALL_SENSOR_FEEDBACK



// *****************************************************************************
//						Read, Write Current Speed
// *****************************************************************************
// These functions are used for ALL feedback models

// returns calculated speed
// used ONLY for serial menus MenuFSM.c
MOTION_PROFILE_SPEED_TYPE CurrentSpeed_Read(enum tagMotors eMotor)
{
	return pguCurrentSpeed[eMotor];
}

#ifdef CALC_AVERAGE_SPEED
// returns calculated average speed
unsigned int AverageSpeed_Read( void )
{
	return pgwAverageSpeed;
}
#endif


// this function is called when a STALL occurs, so that speed will be reported as ZERO
// ==> is this really necessary now? should this just be merged into MotionFSM.c: Init_MotionStats()?
// (only called AFTER Finish_MotionStats() )
void CurrentSpeed_Write(enum tagMotors eMotor, MOTION_PROFILE_SPEED_TYPE NewSpeed)
{
	if (NewSpeed IS ZERO)
	{
		// writing speed to 0 means we are STOPPED, so clear all speed related variables
		fgwLastTimerValue[eMotor] = 0xFFFF;

		pguCurrentSpeed[eMotor] = 0;
		pgsSpeedError[eMotor] = 0;
#ifdef CALC_AVERAGE_SPEED
		fgwSumOfSpeeds[eMotor] = 0;
		fgwSampleCtr[eMotor] = 0;
		fgwLastSpeed[eMotor] = 0;
		pgwAverageSpeed[eMotor] = 0;
		pgnErrorFromAverage[eMotor] = 0;
		pgnErrorFromSample[eMotor] = 0;
#endif

		pgulMoveTotalMSICtr[eMotor] = 0;			// Total motion sensor ticks in move ctr;
		pgwStateMSICtr[eMotor] = 0;

		// note that we do NOT clear:
		//	fglCurrentPosition - so we can maintain position when motion restarts
	}
	else
	{
		pguCurrentSpeed[eMotor] = NewSpeed;
	}
}

// *****************************************************************************
//					Read, Write Current Position/Orientation
// *****************************************************************************
// These functions are used for ALL feedback models


// ==> should this set the value to offset value that would be seen at mechanical center?
void CurrentPosition_Clear(enum tagMotors eMotor)
{
	fglCurrentPosition[eMotor] = (INT32)0;
}

// read the current position counter value
// used to prevent direct access to fglCurrentPosition[]
INT32 CurrentPosition_Read(enum tagMotors eMotor)
{
#if defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_INCLINOMETER_FEEDBACK)
	// read accelerometer values
	if (eMotor IS MOTOR_AZIMUTH)
	{
		if (ReadInclinometerSample(&pgInclination) IS TRUE)
		{
			IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);
		}
		fglCurrentPosition[eMotor] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
	}
#endif
return fglCurrentPosition[eMotor];
}

#if defined(USE_SINGLE_POLAR_AXIS) AND defined(USE_INCLINOMETER_FEEDBACK)
float CurrentAverageAngleDegrees_Read(enum tagMotors eMotor)
{
	// read accelerometer values
	if (eMotor IS MOTOR_AZIMUTH)
	{
		if (ReadInclinometerSample(&pgInclination) IS TRUE)
		{
			IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);
		}
		return (pgAngleAverage.fX_Angle);
	}
	else
	{
		return 0.0;
	}
}
#endif


// update current orientation in ticks
void CurrentPosition_Set(enum tagMotors eMotor, INT32 lNewPosition)
{
	fglCurrentPosition[eMotor] = lNewPosition;
}

// returns mechanical orientation. Does NOT account for stored offset values
void CurrentMechanicalOrientation_Read(PTR_ORIENTATION ptrOrientation)
{

	// read Current Mechanical Orientation in MSI ticks
	ptrOrientation->lAzimuthPositionTicks = CurrentPosition_Read(MOTOR_AZIMUTH);
	ptrOrientation->lElevationPositionTicks = CurrentPosition_Read(MOTOR_ELEVATION);

	ptrOrientation->fAzimuth = ((float)CurrentPosition_Read(MOTOR_AZIMUTH) / AZ_FLOAT_MSI_TICKS_PER_DEGREE);
	ptrOrientation->fElevation = ((float)CurrentPosition_Read(MOTOR_ELEVATION) / EL_FLOAT_MSI_TICKS_PER_DEGREE);

	ptrOrientation->fAzimuth = ConvertMSITicksToDegrees(ptrOrientation->lAzimuthPositionTicks, AXIS_AZIMUTH);
	ptrOrientation->fElevation = ConvertMSITicksToDegrees(ptrOrientation->lElevationPositionTicks, AXIS_ELEVATION);
}


// returns mechanical orientation, offset by stored offset values
void CurrentLocalOrientation_Read(PTR_ORIENTATION ptrOrientation)
{

	// read Current Mechanical Orientation in MSI ticks, add offset converted to ticks
	ptrOrientation->lAzimuthPositionTicks = CurrentPosition_Read(MOTOR_AZIMUTH) + ((INT32)((ptrRAM_SystemParameters->fAZ_Offset * AZ_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET));
	ptrOrientation->lElevationPositionTicks = CurrentPosition_Read(MOTOR_ELEVATION) + ((INT32)((ptrRAM_SystemParameters->fEL_Offset * EL_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET));

	ptrOrientation->fAzimuth = ((float)CurrentPosition_Read(MOTOR_AZIMUTH) / AZ_FLOAT_MSI_TICKS_PER_DEGREE) + ptrRAM_SystemParameters->fAZ_Offset;
	ptrOrientation->fElevation = ((float)CurrentPosition_Read(MOTOR_ELEVATION) / EL_FLOAT_MSI_TICKS_PER_DEGREE) + ptrRAM_SystemParameters->fEL_Offset;

}



// *****************************************************************************
//			M S _ S e t S i m u l a t o r T i c k C t r( )
// *****************************************************************************

#ifdef USE_FEEDBACK_SIMULATOR

// set simlator tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
BOOL MS_SetSimulatorTickCtr(enum tagMotors eMotor)
{
	// bounds check index pgbMotionProfileSpeedIndex[eMotor]
	if ((pgbMotionProfileSpeedIndex[eMotor] < MIN_SPEED_INDEX) OR (pgbMotionProfileSpeedIndex[eMotor] > MAX_SPEED_INDEX[eMotor]))
	{
		RuntimeError(MTN_SENSOR_ERROR_INVALID_TABLE_INDEX);
		return FALSE;
	}

	// set tick counter; this is the expected "speed" value converted from Tcy increments to 5mS increments
	pgwFeedbackSimulatorTickCtr[eMotor] = MotionProfileSpeedAndPWM[eMotor][(pgbMotionProfileSpeedIndex[eMotor] * MOTION_PROFILE_TABLE_WIDTH) + SIMULATOR_5MS_TICK_OFFSET];

}

#endif	// USE_FEEDBACK_SIMULATOR



// end of MotionSensor.c
