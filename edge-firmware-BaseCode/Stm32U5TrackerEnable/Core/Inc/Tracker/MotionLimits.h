// *************************************************************************************************
//										M o t i o n L i m i t s . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Motion Limit calculation definitions and prototypes
//
// *************************************************************************************************
#include"GenericTypeDefs.h"
#include"gsfstd.h"
//-------------------------------------------------------------------------------------------------------
// Function Definitions
//-------------------------------------------------------------------------------------------------------
#ifndef MOTIONLIMITS_H
	#define MOTIONLIMITS_H
#endif

#ifndef MOTIONPHASEFSM_H
	//#error MotionPhaseFSM.h must be #included first
#endif


// requires MotionPhaseFSM.h to define enum tagMoveTypes

BOOL SetMotionLimits(enum tagMotors eMotor, enum tagMoveTypes eMove);

// *********************************************************
//					Motion Definitions
// *********************************************************

// NOTE: Tick counts must be INT32

// Azimuth Drive Values
//#define	AZ_MSI_TICKS_PER_MOTOR_REVOLUTION		((INT32)2)						// Motion Sensor effect ticks per Azimuth motor revolution

// Ticks per degree is:
//		(MOTOR_GB_RATIO * SLEW_GB_RATIO * MSI_HZ_PER_MOTOR_REV) / 360

// these values are based on 273:1 + 73:1 gearboxes
#define	AZ_MSI_TICKS_PER_SLEW_GB_REVOLUTION		((INT32)46800)					// Motion Sensor ticks per Azimuth slew gearbox output revolution
#define	AZ_MSI_TICKS_PER_DEGREE				((INT32)520)						// Motion Sensor ticks per Azimuth 1.0 degree of motion
#define	AZ_FLOAT_MSI_TICKS_PER_DEGREE			(float)520.0						// Motion Sensor ticks per Azimuth 1.0 degree of motion, as a float

#ifdef USE_SINGLE_POLAR_AXIS
	// single polar axis limits azimuth to +-45 degrees
	#define	AZ_DEGREES_MAX						((float)130.0)					// maximum allowable azimuth travel, in degrees
#else
	#define	AZ_DEGREES_MAX						((float)365.0)					// maximum allowable azimuth travel, in degrees (full rotation)
#endif

//#define	AZ_MSI_TICKS_PER_STEP				(AZ_MSI_TICKS_PER_DEGREE/2)		// Motion Sensor ticks per Azimuth 0.5 degree step

#define	AZ_DEG_EOT_MARGIN					((float)5.0)					// total margin from allowable travel (EOT) to Limit Switches in degrees
#define	AZ_TICKS_EOT_MARGIN					((INT32)((AZ_DEG_EOT_MARGIN * AZ_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET))	// margin from allowable travel (EOT) to limit switches, in MSI ticks

#define	AZ_MSI_TICKS_MAX					((INT32)((AZ_DEGREES_MAX * AZ_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET))	// maximum number of MSI ticks from Limit Switch to Limit Switch, 365 degress
#define	AZ_MSI_TICKS_ALLOWABLE					(AZ_MSI_TICKS_MAX - AZ_TICKS_EOT_MARGIN)		// allowable number of ticks, staying away from the end stops by 2.5 degrees (360 degrees)

#define	AZ_SOFT_LIMIT_MSI_TICKS_REVERSE                         -(AZ_MSI_TICKS_ALLOWABLE / 2)	// default reverse travel limit
#define	AZ_SOFT_LIMIT_MSI_TICKS_FORWARD                         (AZ_MSI_TICKS_ALLOWABLE / 2)	// default forward travel limit

// NOTE: SmartTrak has these set to +-38.0 degrees
#define	AZ_SOFT_LIMIT_DEGREES_REVERSE                           -((AZ_DEGREES_MAX - AZ_DEG_EOT_MARGIN) / 2.0)	// default reverse travel limit
#define	AZ_SOFT_LIMIT_DEGREES_FORWARD                           ((AZ_DEGREES_MAX - AZ_DEG_EOT_MARGIN) / 2.0)	// default forward travel limit

#ifdef USE_SINGLE_POLAR_AXIS
	#define	AZ_NIGHTSTOW_POSITION				0.0								// night stow at center
        #define	AZ_WINDSTOW_POSITION				0.0
#else
	#define	AZ_NIGHTSTOW_POSITION				AZ_SOFT_LIMIT_DEGREES_REVERSE,	// night stow at end of travel
#endif

// number of MSI Ticks for each move type
#define	AZ_MSI_TICKS_PER_SLEW					AZ_MSI_TICKS_MAX				// slew to limit stop is actually terminated by limit switch, could be nearly 360 degrees on a slew drive
#define	AZ_DEGREES_PER_SLEW					AZ_DEGREES_MAX					// slew to limit stop is actually terminated by limit switch, could be nearly 360 degrees on a slew drive
#define	AZ_MSI_TICKS_PER_RUN					AZ_MSI_TICKS_ALLOWABLE			// run stays away from the limit stops

// Elevation Drive Values, currently based on using the linear actuator with 58:1 gearbox and 36" total linear travel, assumed to allow 100 angular degrees at the panel
// travel is -3.5 degrees (night stow) to +91.5 degrees
// NOTE: there is currently (23 Aug 13) NOTHING in place to linearize the angular movement of the elevation drive

#if defined(USE_ELEVATION_SLEW_DRIVE)
	#define	EL_MSI_TICKS_PER_ACTUATOR_SLEW		((INT32)20900)					// Motion Sensor ticks per Elevation Slew Drive end-to-end, 100 degrees, 2 ticks per motor revolution
	#define	EL_MSI_TICKS_PER_DEGREE				((INT32)209)					// Motion Sensor ticks per Elevation 1.0 degree of Slew Drive motion
	#define	EL_FLOAT_MSI_TICKS_PER_DEGREE		(float)208.8000					// Motion Sensor ticks per Elevation 1.0 degree of Slew Drive motion, as a float
#elif defined(USE_ELEVATION_LINEAR_DRIVE)
	#define	EL_MSI_TICKS_PER_ACTUATOR_SLEW		((INT32)1656)					// Motion Sensor ticks per Elevation Actuator end-to-end, 36 inches at 64 ticks per inch
	#define	EL_MSI_TICKS_PER_DEGREE				((INT32)28)						// Motion Sensor ticks per Elevation 1.0 degree of output angular motion, assuming 60 degrees total
	#define	EL_FLOAT_MSI_TICKS_PER_DEGREE		(float)27.60					// Motion Sensor ticks per Elevation 1.0 degree of output angular motion, as a float
#endif

#define	EL_DEGREES_MAX							((float)70.0)					// maximum allowable elevation travel, in degrees

#define	EL_DEG_EOT_MARGIN						((float)5.0)					// total margin from allowable travel (EOT) to Limit Switches
#define	EL_TICKS_EOT_MARGIN						((INT32)((EL_DEG_EOT_MARGIN * EL_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET))	// margin from allowable travel (EOT) to limit switches, in MSI ticks

#define	EL_MSI_TICKS_MAX						((INT32)((EL_DEGREES_MAX * EL_MSI_TICKS_PER_DEGREE)	 + CAST_FLOAT_ROUNDING_OFFSET))		// maximum number of MSI ticks from Limit Switch to Limit Switch, 100 degress
#define	EL_MSI_TICKS_ALLOWABLE					(EL_MSI_TICKS_MAX - EL_TICKS_EOT_MARGIN)					// allowable number of ticks, staying away from the end stops by 2.5 degrees (85 degrees)

#define EL_NIGHT_STOW_DEGREES					((float)-3.5)							// night stow elevation position, degrees
#define EL_NIGHT_STOW_TICKS						((7 * EL_MSI_TICKS_PER_DEGREE) / 2)		// night stow elevation position, ticks. math is a kludge to maintain accuracy for fraction (3.5)

#define	EL_SOFT_LIMIT_MSI_TICKS_REVERSE			EL_NIGHT_STOW_TICKS							// default reverse travel limit
#define	EL_SOFT_LIMIT_MSI_TICKS_FORWARD			(EL_MSI_TICKS_ALLOWABLE - EL_SOFT_LIMIT_MSI_TICKS_REVERSE)	// default forward travel limit ==> not correct? Does not match degrees value

#define	EL_SOFT_LIMIT_DEGREES_REVERSE			EL_NIGHT_STOW_DEGREES												// default reverse travel limit
#define	EL_SOFT_LIMIT_DEGREES_FORWARD			((EL_DEGREES_MAX - EL_DEG_EOT_MARGIN) + EL_SOFT_LIMIT_DEGREES_REVERSE)	// default forward travel limit, calculation depends on negative sign of reverse


// number of MSI Ticks for each move type
#define	EL_MSI_TICKS_PER_SLEW					EL_MSI_TICKS_PER_ACTUATOR_SLEW	// slew to limit stop is actually terminated by limit switch, could be nearly 360 degrees on a slew drive
#define	EL_MSI_TICKS_PER_RUN					EL_MSI_TICKS_ALLOWABLE			// run stays away from the limit stops

// output motion of motor gearbox, first of two stage gearbox. For testing ONLY
#define	EL_MOTOR_GB_ROTATION_DEGREES			((float)5.0)					// arbitrary value; no info on gearbox
#define	AZ_MOTOR_GB_ROTATION_DEGREES			((float)5.0)					// slew gearbox output for approximately one rotation of motor gearbox output shaft


// *********************************************************
//				Maximum Travel Limits
// *********************************************************

// these allowable POSITION limits divide the allowable travel by 2, and set 0 in the center.
#define	AZ_ALLOWABLE_POS_FORWARD		(AZ_MSI_TICKS_ALLOWABLE/2)					// maximum allowable travel forward
#define	AZ_ALLOWABLE_POS_REVERSE		(-AZ_MSI_TICKS_ALLOWABLE/2)					// maximum allowable travel reverse

#define	EL_ALLOWABLE_POS_FORWARD		(EL_MSI_TICKS_ALLOWABLE/2)					// maximum allowable travel forward
#define	EL_ALLOWABLE_POS_REVERSE		(-EL_MSI_TICKS_ALLOWABLE/2)					// maximum allowable travel reverse


//-------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------

#ifdef DEFINE_GLOBALS
	GLOBAL_INIT UINT32	pgwMSIAccelerationCount[NUM_MOTORS] = {0, 0};		// count of acceleration MSI ticks in move
	GLOBAL_INIT UINT32	pgulMSIConstantSpeedCount[NUM_MOTORS] = {0, 0};		// count of constant speed MSI ticks; only phase that could exceed 64K ticks
	GLOBAL_INIT UINT32	pgwMSIDecelerationCount[NUM_MOTORS] = {0, 0};		// count of deceleration MSI ticks in move
	#ifdef USE_INCLINOMETER_FEEDBACK
		GLOBAL_INIT float	pgfStartingAngle[NUM_MOTORS] = {0.0, 0.0};
		GLOBAL_INIT float	pgfEndingPositionDegrees[NUM_MOTORS] = {0.0, 0.0};	// end of move, in degrees
	#endif
#else
	GLOBAL UINT32	pgwMSIAccelerationCount[];
	GLOBAL UINT32	pgulMSIConstantSpeedCount[];
	GLOBAL UINT32	pgwMSIDecelerationCount[];
	#ifdef USE_INCLINOMETER_FEEDBACK
		GLOBAL float	pgfStartingAngle[];
		GLOBAL float	pgfEndingPositionDegrees[];
	#endif
#endif

// end of MotionLimits.h
