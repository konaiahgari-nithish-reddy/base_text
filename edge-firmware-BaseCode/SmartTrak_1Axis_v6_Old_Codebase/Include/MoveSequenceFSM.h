// *************************************************************************************************
//									M o v e S e q u e n c e F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Move Sequencer function definitions, data declarations
//
// *************************************************************************************************

#ifndef MOVESEQUENCEFSM_H
	#define MOVESEQUENCEFSM_H
#endif

#ifndef MOTORPWM_H
	#error MotorPWM.h must be #included first
#endif


//----------------------------------------------------------------------
//	Function Prototypes
//----------------------------------------------------------------------

void MoveSequenceFSM(void);
void MoveSequenceTick(void);

char *GetMoveSeqStateString(void);

void SetMoveSequenceStarted(void);
void ClearMoveSequenceStarted(void);
BOOL IsMoveSequenceComplete(void);


//----------------------------------------------------------------------
//	Global Variables
//----------------------------------------------------------------------

typedef struct
{

	float	fSPASetPoint_AZ;			// Float SPA Calculated Azimuth value, limited by soft limits
	float	fSPASetPoint_EL;			// Float SPA Calculated Elevation value, limited by soft limits

	float	fPrevAzimuth;				// Float Previous Azimuth. Units: degrees
	float	fPrevElevation;				// Float Previous Elevation. Units: degrees

	INT32	lPrevAzimuthTicks;			// Prev Azimuth, Units: MSI ticks, converted from degrees
	INT32	lPrevElevationTicks;		// Prev Elevation, Units: MSI ticks, converted from degrees

	float	fNewAzimuth;				// Float New Azimuth. Units: degrees
	float	fNewElevation;				// Float New Elevation. Units: degrees

	INT32	lNewAzimuthTicks;			// New Azimuth, Units: MSI ticks, converted from degrees
	INT32	lNewElevationTicks;			// New Elevation, Units: MSI ticks, converted from degrees

	INT32	lAzimuthMoveTicks;			// Azimuth, Units: MSI ticks, may be positive (FWD) or negative (REV)
	INT32	lElevationMoveTicks;		// Elevation, Units: MSI ticks, may be positive (FWD) or negative (REV)

	enum tagPWMDirection eAzimuthDirection;		// direction of move
	enum tagPWMDirection eElevationDirection;	// direction of move

	INT32	lAzimuthErrorTicks;			// position error, calculated after move, Units: MSI ticks
	INT32	lElevationErrorTicks;		// position error, calculated after move, Units: MSI ticks

	INT32	lAzimuthMoveErrorTicks;		// move error, calculated after move, Units: MSI ticks
	INT32	lElevationMoveErrorTicks;	// move error, calculated after move, Units: MSI ticks

	float	fAzimuthPositionErrorDegrees;	// Position error, calculated after move, accumulates errors
	float	fElevationPositionErrorDegrees;	// Position error, calculated after move, accumulates errors

//	enum tagSunPositionState SunPositionState;	// current state of SPA movement; see SunPosition.h

} SPA_MOVE_INFO, *PTR_SPA_MOVE_INFO;


//----------------------------------------------------------------------
//	Global Variables
//----------------------------------------------------------------------

#ifdef DEFINE_GLOBALS
	ARRAY GLOBAL_INIT float	pgfMS_MoveDistanceDegrees[NUM_MOTORS] = {0.0, 0.0};			// move sequence move distance
	ARRAY GLOBAL_INIT float	pgfMS_StepDistanceDegrees[NUM_MOTORS] = {0.0, 0.0};			// move sequence move distance
        ARRAY GLOBAL_INIT float	pgfMS_ManDistanceDegrees[NUM_MOTORS] = {0.0, 0.0};			// move sequence move distance
	GLOBAL_INIT  BOOL bFindEndPointsHasRun = FALSE;
        GLOBAL_INIT  BYTE bTrackerDirection = 0;
#else
	ARRAY GLOBAL float	pgfMS_MoveDistanceDegrees[];
	ARRAY GLOBAL float	pgfMS_StepDistanceDegrees[];
        ARRAY GLOBAL float	pgfMS_ManDistanceDegrees[];
	GLOBAL BOOL bFindEndPointsHasRun;
        GLOBAL BYTE bTrackerDirection;
#endif


// end of MoveSequenceFSM.h
