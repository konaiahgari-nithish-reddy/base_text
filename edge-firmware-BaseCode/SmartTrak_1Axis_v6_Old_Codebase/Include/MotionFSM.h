// *************************************************************************************************
//										M o t i o n F S M . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Control FSM function prototypes, definitions
//
// *************************************************************************************************

#ifndef MOTIONFSM_H
	#define MOTIONFSM_H
#endif

#ifndef MOTORPWM_H
	#error MotorPWM.h must be #included first
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// Motion Control FSM states
enum tagMotionStates
{
    ST_MOTION_INIT,				// initial state, only at power up

    ST_MOTION_STOPPED,			// all motion stopped

    ST_MOTION_ACCELERATE,		// accelerate by increasing PWM
    ST_MOTION_ACC_INCREASE_PWM,	// increase PWM to speed up
    ST_MOTION_ACC_DECREASE_PWM,	// decrease PWM to slow down (?)
    ST_MOTION_CONSTANT_SPEED,	// constant speed, constant PWM
    ST_MOTION_CON_INCREASE_PWM,	// accelerate PWM to overcome load and maintain constant speed
    ST_MOTION_CON_DECREASE_PWM,	// decrease PWM to slow down (?)
    ST_MOTION_DECELERATE,		// decelerate by reducing PWM
    ST_MOTION_MINIMUM_PWM,		// constant speed at minimum PMW
	ST_MOTION_COASTING,

    ST_MOTION_SOFT_STALL,		// stall recovery state
    ST_MOTION_HARD_STALL		// stall error state

};


// Motion Type
enum tagMotionTypes
{
	MOTION_INIT,				// initial state, only at power up

	MOTION_STOPPED,				// motion has actually STOPPED (quadrature encoder timeout)
	MOTION_STARTING,			// PWM is on, but no MSI ticks yet
	MOTION_POWERED,				// motor is ON
	MOTION_COASTING,			// motor is OFF, some coasting MAY occur
	MOTION_BRAKING,				// motor is OFF, PWM shorted to act as electric brake

	MOTION_STALLED				// system has STALLED; no exit
};


// Motion Phase, to keep track of what part of a move is executing
// The Phases group together related states within the Motion FSM, to allow external code (such as the Quadrature Encoder Tick handler)
// to treat multiple Motion FSM states as a single state - a motion Phase
enum tagMotionPhases
{
	PHASE_INIT,					// initial state, only at power up

	PHASE_STOPPED,				// motion has actually STOPPED (quadrature encoder timeout), includes STALLED

	PHASE_ACCELERATION,			// accelerating speed
	PHASE_CONSTANT_SPEED,		// constant speed operation
	PHASE_DECELERATION,			// decelerating
    PHASE_MINIMUM_PWM,			// constant speed at minimum PMW
	PHASE_COASTING,				// motor is OFF, some coasting MAY occur

	PHASE_OPEN_LOOP				// used ONLY for open loop operation
};

//-------------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------------

void SetMotionStarted(enum tagMotors eMotor);
void ClearMotionStarted(enum tagMotors eMotor);
BOOL IsMotionComplete(enum tagMotors eMotor);

void MotionFSM(enum tagMotors eMotor);
const char *GetMotionStateString(enum tagMotors eMotor);
enum tagMotionStates GetMotionState(enum tagMotors eMotor);
void ResetMotionFSM(void);


//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#ifdef DEFINE_GLOBALS
	GLOBAL_INIT enum tagMotionTypes pgeMotionType[NUM_MOTORS] = {MOTION_INIT, MOTION_INIT};
	GLOBAL_INIT enum tagMotionPhases pgeMotionPhase[NUM_MOTORS] = {PHASE_INIT, PHASE_INIT};
	GLOBAL_INIT INT8S pgcDutyCycleCorrection[NUM_MOTORS] = {0, 0};

	GLOBAL_INIT enum tagPWMConfigs pgeStallRecoveryPWMConfig[NUM_MOTORS] = {PWM_CONFIG_UNKNOWN, PWM_CONFIG_UNKNOWN};

	// these are active MSI (Hall Effect Interrupt) tick counters, used during a move
	// the currently used gear ratios should allow all values to fit in a WORD

	GLOBAL_INIT WORD	pgwMSI_AccelerationCtr[NUM_MOTORS] = {0, 0};			// MSI ticks for each of the motion phases
	GLOBAL_INIT WORD	pgulMSI_ConstantSpeedCtr[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT WORD	pgwMSI_DecelerationCtr[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT WORD	pgwMSI_MinimumPWMCtr[NUM_MOTORS] = {0, 0};
	GLOBAL_INIT WORD	pgwMSI_CoastCtr[NUM_MOTORS] = {0, 0};
	//GLOBAL_INIT WORD	pgwMSI_TotalCtr[NUM_MOTORS] = {0, 0};

	#ifdef USE_FEEDBACK_SIMULATOR
		GLOBAL_INIT WORD	pgwFeedbackSimulatorTickCtr[NUM_MOTORS] = {0, 0};
	#endif
#else
	GLOBAL enum tagMotionTypes pgeMotionType[];
	GLOBAL enum tagMotionPhases pgeMotionPhase[];
	GLOBAL INT8S pgcDutyCycleCorrection[];

	GLOBAL enum tagPWMConfigs pgeStallRecoveryPWMConfig[];

	GLOBAL WORD	pgwMSI_AccelerationCtr[];			// MSI ticks for each of the motion phases
	GLOBAL WORD	pgulMSI_ConstantSpeedCtr[];			// should this be UINT32?
	GLOBAL WORD	pgwMSI_DecelerationCtr[];
	GLOBAL WORD	pgwMSI_MinimumPWMCtr[];
	GLOBAL WORD	pgwMSI_CoastCtr[];
	//GLOBAL WORD	pgwMSI_TotalCtr[];

	#ifdef USE_FEEDBACK_SIMULATOR
		GLOBAL WORD	pgwFeedbackSimulatorTickCtr[];
	#endif
#endif


// end of MotionFSM.h
