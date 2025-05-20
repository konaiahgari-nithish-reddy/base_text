// *************************************************************************************************
//										M o t o r P W M . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motor PWM Init, Control Definitions
//
// *************************************************************************************************
#include"GenericTypeDefs.h"
#include"gsfstd.h"

#ifndef MOTORPWM_H
	#define MOTORPWM_H
#endif

// must be included AFTER MotionProfile.h

//#define GLOBAL
//#define 
//----------------------------------------------------------------------
//	Definitions
//----------------------------------------------------------------------

// NOTE: these definitions are separate from the motor definitions, because they are PWM settings

// the configurations indicate how the PWM outputs have been setup to drive the motor
// PWM configuration is set only once per motion phase
enum tagPWMConfigs
	{
		PWM_CONFIG_REVERSE = -1,
		PWM_CONFIG_STOPPED = 0,
		PWM_CONFIG_FORWARD,
		PWM_CONFIG_BRAKE,						// Toshiba calls this 'Short Brake'
//		PWM_CONFIG_STANDBY,						// motor controller in standby (1uA current)
		PWM_CONFIG_UNKNOWN						// error value
	};

// PWM direction is used for every call to adjust the PWM, which may be many times per motion phase
enum tagPWMDirection
	{
		PWM_DIR_REVERSE = -1,
		PWM_DIR_STOPPED = 0,
		PWM_DIR_FORWARD,
		PWM_DIR_UNKNOWN							// error value
	};


#if defined(USE_PWM_DUTY_CYCLE)
	#define	PWM_DUTY_CYCLE_OFF					(UINT8)0
	#define	PWM_DUTY_CYCLE_MIN					28	// not a constant? (MotionProfileSpeedAndPWM[(MIN_SPEED_INDEX * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET])
	//#define	PWM_DUTY_CYCLE_MIN				(const WORD (MotionProfileSpeedAndPWM[(MIN_SPEED_INDEX * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET]))
	//#define	PWM_DUTY_CYCLE_MAX				98	// not a constant? (MotionProfileSpeedAndPWM[(MAX_SPEED_INDEX * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET])
	#define	PWM_DUTY_CYCLE_MAX(x)				(UINT8)MotionProfileSpeedAndPWM[x][(MAX_SPEED_INDEX(x) * MOTION_PROFILE_TABLE_WIDTH) + PWM_VALUE_OFFSET]
	#define	PWM_DUTY_CYCLE_HALF(x)				(UINT8)((PWM_DUTY_CYCLE_MAX(x) + PWM_DUTY_CYCLE_MIN(x))/2)				// middle of usable range
#elif defined(USE_PWM_ON_OFF)
	#define	PWM_DUTY_CYCLE_OFF					(UINT8)0
	#define	PWM_DUTY_CYCLE_MIN					100
	#define	PWM_DUTY_CYCLE_MAX(x)				100
	#define	PWM_DUTY_CYCLE_HALF(x)				100
#endif

// PWM starting and increment values used during SOFT_STALL stall recovery
// Stopped Motion Stall Recovery is based on simply ramping PWM values, NOT walking the motion profile table
#define	PWM_STALL_RECOVERY_MIN				PWM_DUTY_CYCLE_HALF
//#define	PWM_STALL_RECOVERY_MAX			PWM_DUTY_CYCLE_MAX
//#define	PWM_STALL_RECOVERY_INCREMENT	10
//#define	PWM_STALL_RECOVERY_DECREMENT	20					// in stall recovery, we decrement to (near) 0, there is no minimum PWM phase

#define PWM_RATE						10000				// 10KHz
#define	PWM_PERIOD						((SystemCoreClock)  / PWM_RATE)	// in clock ticks  <sek> 18 Mar 13 changed from SYS_FREQ due to divider change


//----------------------------------------------------------------------
//	Motor PWM functions
//----------------------------------------------------------------------

void PWM_Init(enum tagMotors eMotor);

//void PWM_5mSecTick(void);

void PWM_SetConfig(enum tagMotors eMotor, enum tagPWMConfigs ePWMConfig);
BOOL PWM_SetDutyCycle(enum tagMotors eMotor, UINT8 bDutyCycle);

//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#ifdef DEFINE_GLOBALS
	GLOBAL_INIT enum tagPWMConfigs		pgePWMConfig[NUM_MOTORS] = {PWM_CONFIG_UNKNOWN, PWM_CONFIG_UNKNOWN};
	GLOBAL_INIT enum tagPWMDirection	pgePWMDirection[NUM_MOTORS] = {PWM_DIR_UNKNOWN, PWM_DIR_UNKNOWN};
	GLOBAL_INIT BYTE			pgbPWMDutyCycleMin[NUM_MOTORS] = {PWM_DUTY_CYCLE_MIN, PWM_DUTY_CYCLE_MIN};
	GLOBAL_INIT BYTE			pgbPWMDutyCycle[NUM_MOTORS] = {PWM_DUTY_CYCLE_OFF, PWM_DUTY_CYCLE_OFF};
	// unused GLOBAL_INIT WORD pgwPWMDutyCycleMax[NUM_MOTORS] = PWM_DUTY_CYCLE_OFF;
	// unused GLOBAL_INIT WORD pgwPWMDutyCycleIncrement[NUM_MOTORS] = 0;
	// unused GLOBAL_INIT WORD pgwPWMDutyCycleDecrement[NUM_MOTORS] = 0;
#else
	GLOBAL enum tagPWMConfigs			pgePWMConfig[];
	GLOBAL enum tagPWMDirection			pgePWMDirection[];
	GLOBAL BYTE							pgbPWMDutyCycleMin[];
	GLOBAL BYTE							pgbPWMDutyCycle[];
	// unused GLOBAL WORD pgwPWMDutyCycleMax[];
   	// unused GLOBAL WORD pgwPWMDutyCycleIncrement[];
	// unused GLOBAL WORD pgwPWMDutyCycleDecrement[];
#endif


// end of MotorPWM.h

