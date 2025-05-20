// *************************************************************************************************
//										M o t i o n P h a s e F S M . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Phase and Command processing FSM definitions
//
// *************************************************************************************************
#ifndef MOTIONPHASEFSM_H
	#define MOTIONPHASEFSM_H
#endif

void MotionPhaseFSM(enum tagMotors eMotor);
const char *GetMotionPhaseStateString(enum tagMotors eMotor);

void SetCommandStarted(enum tagMotors eMotor);
void ClearCommandStarted(enum tagMotors eMotor);
BOOL IsCommandComplete(enum tagMotors eMotor);


// Command Level Move Types, used to tell SetMotionLimits() what type of move to define
// this list actually describes all of the basic move types; everything else is a sequence of one or more of these
enum tagMoveTypes
{
	MOVE_NONE,				// no move, stopped

	MOVE_RUN_FWD,			// move [specified distance] forward
	MOVE_RUN_REV,			// move [specified distance] reverse

	MOVE_SLEW_FWD_TO_END,	// move maximum distance forward until end of travel sense
	MOVE_SLEW_REV_TO_END,	// move maximum distance reverse until end of travel sense

	MOVE_STALL_RECOVERY,	// short, fast, opposite move to recover from a stall
	MOVE_COMPLETE = 256		// flag to indicate previous move is complete; used for move sequences

};

#ifdef DEFINE_GLOBALS
	ARRAY GLOBAL_INIT enum tagMoveTypes eMoveType[NUM_MOTORS] = {MOVE_NONE, MOVE_NONE};
	ARRAY GLOBAL_INIT INT32 pglMoveDistanceTicks[NUM_MOTORS] = {0L, 0L};
	ARRAY GLOBAL_INIT float pgfMoveDistanceDegrees[NUM_MOTORS] = {0.0, 0.0};
#else
	ARRAY GLOBAL enum tagMoveTypes eMoveType[];
	ARRAY GLOBAL INT32 pglMoveDistanceTicks[];
	ARRAY GLOBAL float pgfMoveDistanceDegrees[];
#endif

// end of MotionPhaseFSM.h
