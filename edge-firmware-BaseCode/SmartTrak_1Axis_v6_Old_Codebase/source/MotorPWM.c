// *************************************************************************************************
//										M o t o r P W M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motor PWM Init, Control
//
// *************************************************************************************************

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include <string.h>				// Microchip string functions
								// see hlpC18.chm online help for so-called documentation
#include <ctype.h>				// tolower()

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "EventFlags.h"			// event flag definitions and globals

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions

#include "Stubs.h"


#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagPWMErrors
{
	PWM_ERROR_NONE = PWM_ERROR_BASE,
	PWM_ERROR_UNEXPECTED_TICK,					// 1 unexpected timer tick event
	PWM_ERROR_UNEXPECTED_EVENT,					// 2 unexpected event
	PWM_ERROR_INVALID_STATE,					// 3 not a valid state
	PWM_ERROR_INVALID_SUBSTATE,					// 4 not a valid state
	PWM_ERROR_INVALID_CHANNEL,					// 5 not a valid channel number
	PWM_ERROR_INVALID_DIRECTION,				// 6 not a valid direction
	PWM_ERROR_INVALID_DUTY_CYCLE_HIGH,			// 7 not a valid duty cycle, high
	PWM_ERROR_INVALID_DUTY_CYCLE_LOW,			// 8 not a valid duty cycle, low
	PWM_ERROR_INVALID_DUTY_CYCLE,				// 9 not a valid duty cycle, general
	PWM_ERROR_CONFIG_DIR_MISMATCH,				// A current config and requested direction do not match
	PWM_ERROR_INVALID_CONFIGURATION_CHANGE,		// B change from current to new configuration is not allowed
	PWM_ERROR_REDUNDANT_CONFIGURATION_CHANGE,	// C change from current to new configuration is redundant (no change!)

	PWM_FSM_UNPROCESSED_EVENT = PWM_ERROR_BASE + 0x0F
};

//----------------------------------------------------------------------
// System variables
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// System functions
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Code functions
//----------------------------------------------------------------------

// *************************************************************************************************
//									P W M _ I n i t ( )
// *************************************************************************************************

// Useful Reference Material:
//
//		PIC32 Family Reference Manual, Section 16. Output Compare
//			version DS61111E, 16.3.3 Pulse Width Modulation Mode, page 16-26
//			version DS61111D, 16.3.3 Pulse Width Modulation Mode, page 16-54
//
//			NOTE: version D is older, but has much more useful information on the RELATED registers, such as timers
//			NOTE: the PIC32MX Family Reference Manual documents are much older, dating from 2008

//	The following registers control the operation of the OC module:



// According to the PIC32 Family Reference Manual, Section 16. Output Compare 16.3.3 Pulse Width Modulation Mode, page 16-54,
// the required operations to setup  the Output Compare module for PWM operation:
//	1. Set the PWM period by writing to the selected timer period register (PRy).
//	2. Set the PWM duty cycle by writing to the OCxRS register.
//	3. Write the OxCR register with the initial duty cycle.
//	4. Enable interrupts, if required, for the timer and Output Compare modules. The output
//			compare interrupt is required for PWM Fault pin utilization.
//	5. Configure the Output Compare module for oneof two PWM Operation modes by writing
//			to the Output Compare mode bits, OCM<2:0> (OCxCON<2:0>).
//	6. Set the TMRy prescale value and enable the time base by setting TON (TxCON<15>) = 1

//	Note:
//		The OCxR register should be initialized before the Output Compare module is first 
//		enabled. The OCxR register becomes a read-only duty cycle register when the 
//		module is operated in the PWM modes. The value held in OCxR will become the 
//		PWM duty cycle for the first PWM period. The contents of the duty cycle buffer 
//		register, OCxRS, will not be transferred into OCxR until a time base period match occurs

void PWM_Init(enum tagMotors eMotor)
{

	InitMotorDrive();				// initializes pins associated with motor drives, but does NOT enable the driver

	// initialize and enable the PWM for all 4 outputs (two per motor)
	// BUT	set the compare values to 0
	//		set the duty cycle to 0 (redundantly sets compare values to 0)
	//		set the configuration to STOPPED

	// this may NOT be the optimal way to do this, but it DOES appear to work
	// perhaps we should be enabling and disabling the PWM, so only the active one are enabled?

	#ifndef _lint		// too many complex PC-Lint errors
		if (eMotor IS MOTOR_AZIMUTH)
		{
			// why did the sample code used for this include OC_LOW_HIGH?
			//      <<----------------------------OC1CON--------------------->>  OC1RS      OC1R
			/*     | 16 bit Mode    | Timer2 source |   PWM w/o Fault Input   , S Compare value, Compare value*/
			OpenOC1(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC1CON |= (OC_ON);					// enable output compare 1

			OpenOC2(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC2CON |= (OC_ON);					// enable output compare 2

                        OpenOC5(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC5CON |= (OC_ON);					// enable output compare 5

		}
		else if (eMotor IS MOTOR_ELEVATION)
		{
			//      <<----------------------------OC3CON--------------------->>  OC3RS      OC3R
			/*     | 16 bit Mode    | Timer2 source |   PWM w/o Fault Input   , S Compare value, Compare value*/
			OpenOC3(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC3CON |= (OC_ON);					// enable output compare 3

			OpenOC4(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC4CON |= (OC_ON);					// enable output compare 4

                        OpenOC5(OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,  0x0000, 0x0000);
			OC5CON |= (OC_ON);					// enable output compare 2

		}

		/* Open Timer2 with Period register value */
		//		   T2CON   PR2			TMR2 = 0
		OpenTimer2(T2_ON, PWM_PERIOD);
	#endif	//_lint

	// see Timer.c:InitializeTimers() for TMR2 and PR2 register initialization
	// set PWM output PWM_SetDutyCycleduty cycle, specified as a percentage of the PWM period
	IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, PWM_DUTY_CYCLE_OFF);

	// set Motor Driver configuration 
	// NOTE: setting PWM to STOPPED requires setting DutyCycle = 0 to force MOTOR_CTRL_PWM1 output LOW
	PWM_SetConfig(eMotor, PWM_CONFIG_STOPPED);

	EnableMotorDrive();						// enable motor driver ICs (MC33926)

}


// *************************************************************************************************
//									P W M   H a n d l e r s
// *************************************************************************************************

// PWM_SetConfig() is called at most once for each motion phase to setup the PWM drive outputs
// PWM_CONFIG_FORWARD and PWM_CONFIG_REVERSE initialize the Motion Stats and enable Motion Sensor interrupts
// PWM_CONFIG_STANDBY disables Motion Sensor interrupts

void PWM_SetConfig(enum tagMotors eMotor, enum tagPWMConfigs ePWMConfig)
{
	// in ALL cases, we want to DISABLE the PWM until the duty cycle has been set by a call to PWM_SetDutyCycle()
	// what does the PWM module output do when disabled? Does it turn back into an floating input? NOT APPROPRIATE.

	//???											// disable the PWM timebase to stop the PWM

	#ifndef _lint		// too many complex PC-Lint errors
		// clear the PWM Timer count register, so the next cycle will start at the beginning of a PWM cycle
		TMR2 = 0x00;
	#endif

	
	// *******************************************
	//		Check for Allowed Transition
	// *******************************************

 	// make sure the requested new configuration is an allowable change from the current configuration
	switch(pgePWMConfig[eMotor])
		{
		case PWM_CONFIG_UNKNOWN:					// initial state
		case PWM_CONFIG_REVERSE:
		case PWM_CONFIG_FORWARD:
		case PWM_CONFIG_BRAKE:
			if (ePWMConfig IS PWM_CONFIG_STOPPED)
				{
				break;
				}
			else
				{
				// not an allowable change, so FORCE an allowable change
				ePWMConfig = PWM_CONFIG_STOPPED;
				RuntimeError(PWM_ERROR_INVALID_CONFIGURATION_CHANGE);
				}
			break;

		case PWM_CONFIG_STOPPED:
			if ((ePWMConfig IS PWM_CONFIG_REVERSE) OR (ePWMConfig IS PWM_CONFIG_FORWARD) OR (ePWMConfig IS PWM_CONFIG_BRAKE) /* OR (ePWMConfig IS PWM_CONFIG_STANDBY)*/)
				{
				break;
				}
			else
				{
				// not an allowable change, so FORCE an allowable change
				ePWMConfig = PWM_CONFIG_STOPPED;
				RuntimeError(PWM_ERROR_REDUNDANT_CONFIGURATION_CHANGE);
				}
			break;

		#ifdef NOT_USED
			case PWM_CONFIG_STANDBY:
				if (ePWMConfig IS PWM_CONFIG_STOPPED)
					{
					break;
					}
				else
					{
					// not an allowable change, so FORCE an allowable change
					ePWMConfig = PWM_CONFIG_STOPPED;
					RuntimeError(PWM_ERROR_INVALID_CONFIGURATION_CHANGE);
					}
				break;
		#endif

		default:
			RuntimeError(PWM_ERROR_INVALID_DIRECTION);
			pgePWMDirection[eMotor] = PWM_DIR_UNKNOWN;		// set project global PWM direction flag

			// not an allowable change, so FORCE an allowable change
			ePWMConfig = PWM_CONFIG_STOPPED;
			RuntimeError(PWM_ERROR_INVALID_CONFIGURATION_CHANGE);
			break;
		}


	// *******************************************
	//			Set New Configuration
	// *******************************************

	switch(ePWMConfig)
		{
		case PWM_CONFIG_REVERSE:
			pgePWMDirection[eMotor] = PWM_DIR_REVERSE;		// set project global PWM direction flag
			pgeMotionType[eMotor] = MOTION_STARTING;		// motion is STARTING, will be MOTION_POWERED after first MSI interrupt

			MotionSensor_EnableInt(eMotor);					// Enable  Motion Sensor (Input Capture) interrupts
			Init_MotionStats(eMotor);						// initialize motion stats and info
			break;

		case PWM_CONFIG_FORWARD:
			pgePWMDirection[eMotor] = PWM_DIR_FORWARD;		// set project global PWM direction flag
			pgeMotionType[eMotor] = MOTION_STARTING;		// motion is STARTING, will be MOTION_POWERED after first MSI interrupt

			MotionSensor_EnableInt(eMotor);					// Enable  Motion Sensor (Input Capture) interrupts
			Init_MotionStats(eMotor);						// initialize motion stats and info
			break;

		case PWM_CONFIG_STOPPED:
			//MOTOR_CTRL_PWM1 =								// PWM output HI
			IGNORE_RETURN_VALUE PWM_SetDutyCycle(eMotor, PWM_DUTY_CYCLE_OFF);	// PWM OFF sets output LOW (==> perhaps not what we want?)

			// we do not update pgePWMDirection because PWM_DIR_STOPPED is NOT a direction
			pguCurrentSpeed[eMotor] = 0;					// speed must now be 0
			pgeMotionType[eMotor] = MOTION_COASTING;		// motion is COASTING  (WHY?)
			break;

		case PWM_CONFIG_BRAKE:
			// Brake enables the two LOW side MOSFETs to short the motor
			// NOTE: we have checked above to make sure the current configuration is PWM_CONFIG_REVERSE OR PWM_CONFIG_FORWARD
			// PWM output can be HI or LOW

			// we do not update pgePWMDirection because PWM_DIR_STOPPED is NOT a direction
			pguCurrentSpeed[eMotor] = 0;					// speed must now be 0
			pgeMotionType[eMotor] = MOTION_BRAKING;			// motion is BRAKING
			break;

		#ifdef NOT_USED
			case PWM_CONFIG_STANDBY:
				// Brake enables the two LOW side MOSFETs to short the motor
				// NOTE: we have checked above to make sure the current configuration is PWM_CONFIG_REVERSE OR PWM_CONFIG_FORWARD
				// PWM output can be HI or LOW
				//MOTOR_CTRL_STBY = 0;							// power off the motor controller

				// we do not update pgePWMDirection because PWM_DIR_STOPPED is NOT a direction
				pguCurrentSpeed[eMotor] = 0;					// speed must now be 0
				pgeMotionType[eMotor] = MOTION_STOPPED;			// motion is BRAKING
				MotionSensor_DisableInt(eMotor);				// disable Motion Sensor (Input Capture) interrupts
				break;
		#endif

		case PWM_CONFIG_UNKNOWN:					// meaningless error
		default:
			// these are errors, and should have been covered above..
			RuntimeError(PWM_ERROR_INVALID_DIRECTION);
			pgePWMDirection[eMotor] = PWM_DIR_UNKNOWN;		// set project global PWM direction flag
			pgePWMConfig[eMotor] = PWM_CONFIG_UNKNOWN;		// set project global PWM configuration flag
			MotionSensor_DisableInt(eMotor);				// disable Motion Sensor (Input Capture) interrupts
			return;
		}

	// save the new configuration to the project global PWM configuration flag
	pgePWMConfig[eMotor] = ePWMConfig;						// set project global PWM configuration flag


	//???												// enable the PWM timebase to start the PWM

}


// set PWM output duty cycle, specified as a percentage of the PWM period (0 to 100)
BOOL PWM_SetDutyCycle(enum tagMotors eMotor, UINT8 bDutyCycle)
{

	BOOL bRetVal = TRUE;

	// the passed parameter duty cycle bDutyCycle ranges from 0 (off) to 100 (full on), although a range of values from 1 to pgbPWMDutyCycleMin (about 20) is not useful/allowed.

	//		PIC32 Family Reference Manual, Section 16. Output Compare
	//			version DS61111E, 16.3.3 Pulse Width Modulation Mode, page 16-28

	//	Some important boundary parameters of the PWM duty cycle include the following:
	//		• If the duty cycle register OCxR is loaded with 0x0000, the OCx pin will remain low (0% duty cycle)
	//		• If OCxR is greater than PRy (timer period register), the pin will remain high (100% duty cycle)
	//		• If OCxR is equal to PRy, the OCx pin will be low for one time base count value and high for all other count values

	// **********************************
	// bounds check duty cycle
	// **********************************
	// maximum PWM value check
	if (bDutyCycle > PWM_DUTY_CYCLE_MAX(eMotor))
		{
		RuntimeError(PWM_ERROR_INVALID_DUTY_CYCLE_HIGH);
		bDutyCycle = PWM_DUTY_CYCLE_MAX(eMotor);

		BITSET(efMotionEvents[eMotor], EF_MOTION_MAXIMUM_PWM);
		bRetVal = FALSE;									// not a valid value
		}

	// minimum PWM value check
	if ((bDutyCycle < pgbPWMDutyCycleMin[eMotor]) AND (bDutyCycle IS_NOT 0))
		{
		RuntimeError(PWM_ERROR_INVALID_DUTY_CYCLE_LOW);
		bDutyCycle = pgbPWMDutyCycleMin[eMotor];

		BITSET(efMotionEvents[eMotor], EF_MOTION_MINIMUM_PWM);
		bRetVal = FALSE;									// not a valid value
		}

	// **********************************
	//		update duty cycle
	// **********************************
	#if defined(USE_PWM_DUTY_CYCLE)
		pgbPWMDutyCycle[eMotor] = bDutyCycle;				// keep track of current duty cycle; for testing only
	#elif defined(USE_PWM_ON_OFF)
		if (bDutyCycle IS_NOT 0)
			bDutyCycle =  PWM_DUTY_CYCLE_MAX(eMotor);		// On/Off operation uses 0, 100% settings ONLY
		
		pgbPWMDutyCycle[eMotor] = bDutyCycle;				// keep track of current duty cycle; for testing only
	#endif

	#ifndef _lint		// too many complex PC-Lint errors in hardware access
		if (eMotor IS MOTOR_AZIMUTH)
		{
			if (bDutyCycle IS 0)							// make sure we do not divide by 0!
			{
				SetDCOC1PWM((UINT32)0);						// writes to OC1RS
				SetDCOC2PWM((UINT32)0);						// writes to OC1RS
                                SetDCOC5PWM((UINT32)0);	
			}
			else
			{
				if (pgePWMConfig[eMotor] IS PWM_CONFIG_FORWARD)
				{
					SetDCOC1PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
					SetDCOC2PWM((UINT32)0);						// writes to OC1RS
                                        SetDCOC5PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
				}
				else if (pgePWMConfig[eMotor] IS PWM_CONFIG_REVERSE)
				{
					SetDCOC1PWM((UINT32)0);
					SetDCOC2PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
                                        SetDCOC5PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
				}
				else	// not a valid running config
				{
					SetDCOC1PWM((UINT32)0);
					SetDCOC2PWM((UINT32)0);
                                        SetDCOC5PWM((UINT32)0);
				}
			}
		}
		else if (eMotor IS MOTOR_ELEVATION)
		{
			// writes to OC1RS
			if (bDutyCycle IS 0)							// make sure we do not divide by 0!
			{
				SetDCOC3PWM((UINT32)0);
				SetDCOC4PWM((UINT32)0);
                                SetDCOC5PWM((UINT32)0);
			}
			else
			{
				if (pgePWMConfig[eMotor] IS PWM_CONFIG_FORWARD)
				{
					SetDCOC3PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
					SetDCOC4PWM((UINT32)0);
                                        SetDCOC5PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
				}
				else if (pgePWMConfig[eMotor] IS PWM_CONFIG_REVERSE)
				{
					SetDCOC3PWM((UINT32)0);
					SetDCOC4PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
                                        SetDCOC5PWM((UINT32)(PWM_PERIOD / 100) * (UINT32)bDutyCycle);
				}
				else	// not a valid running config
				{
					SetDCOC3PWM((UINT32)0);
					SetDCOC4PWM((UINT32)0);
                                        SetDCOC5PWM((UINT32)0);
				}
			}
		}
		else
		{
			bRetVal = FALSE;								// not a valid motor ID
		}
	#endif	// _lint

	return bRetVal;
}


#ifdef USE_FEEDBACK_SIMULATOR
	// the serial menu includes some commands that allow directly setting the PWM value, rather than reading it from MotionProfileSpeedAndPWM[ ]
	// if this is used in combination with the Feedback Simulator, the simulator code needs the index into MotionProfileSpeedAndPWM[ ]
	// in order to produce simulated Motion Sensor interrupts at the appropriate rate
	// This function takes the PWM value, finds where it fits into the MotionProfileSpeedAndPWM[ ] table, and sets the global index

	BOOL PWM_FindMotionProfileSpeedAndPWMIndex(enum tagMotors eMotor, unsigned int nDutyCycle)
	{
		int i;

		// bounds check duty cycle
		if (nDutyCycle < MotionProfileSpeedAndPWM[eMotor][(MIN_SPEED_INDEX * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET])
		{
			RuntimeError(PWM_ERROR_INVALID_DUTY_CYCLE_LOW);
			return FALSE;
		}

		if (nDutyCycle > MotionProfileSpeedAndPWM[eMotor][(MAX_PWM_INDEX(eMotor) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET])
		{
			RuntimeError(PWM_ERROR_INVALID_DUTY_CYCLE_HIGH);
			return FALSE;
		}

		// walk through the MotionProfileSpeedAndPWM[eMotor][] and find the table index where the input duty cycle fits
		for(i = MIN_SPEED_INDEX; i <= MAX_PWM_INDEX(eMotor); i++)
		{
			if((MotionProfileSpeedAndPWM[eMotor][(i * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET] <= nDutyCycle)
				AND(MotionProfileSpeedAndPWM[eMotor][((i+1) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET] >= nDutyCycle))
			{
				// found an appropriate table index
				pgbMotionProfileSpeedIndex[eMotor] = i;
				return TRUE;
			}
		}

		RuntimeError(PWM_ERROR_INVALID_DUTY_CYCLE);
		return FALSE;
	}
#endif		// USE_FEEDBACK_SIMULATOR


// end of MotorPWM.c

