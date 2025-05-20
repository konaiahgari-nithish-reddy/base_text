// *************************************************************************************************
//										U p d a t e P a r a m e t e r s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Persistent parameter update, used for menus and remote commands
//
// *************************************************************************************************


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagParameter
{
	PARAMETER_NONE,

	// System Parameters: unit location
	PARAMETER_LATITUDE,
	PARAMETER_LONGITUDE,
	PARAMETER_ALTITUDE,
	PARAMETER_REFRACTION,
	PARAMETER_TIMEZONE,
	PARAMETER_TRACKING_MODE,

	// System Parameters: Azimuth
	PARAMETER_AZ_OFFSET,
	PARAMETER_AZ_SOFT_LIMIT_REV,
	PARAMETER_AZ_SOFT_LIMIT_FWD,
	PARAMETER_AZ_DEAD_BAND,
	PARAMETER_AZ_NIGHT_STOW_THRESHOLD,
	PARAMETER_AZ_NIGHT_STOW_POS,
	PARAMETER_AZ_WIND_STOW_POS,

	// System Parameters: Elevation
	PARAMETER_EL_OFFSET,
	PARAMETER_EL_SOFT_LIMIT_REV,
	PARAMETER_EL_SOFT_LIMIT_FWD,
	PARAMETER_EL_DEAD_BAND,
	PARAMETER_EL_NIGHT_STOW_THRESHOLD,
	PARAMETER_EL_NIGHT_STOW_POS,
	PARAMETER_EL_WIND_STOW_POS,

	// Backtracking
	#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_BACKTRACKING)
		PARAMETER_BACKTRACKING_ENABLED,
		PARAMETER_PANEL_SHADOW_START_ANGLE,
		PARAMETER_SUN_SHADOW_START_ANGLE,
		PARAMETER_SUN_SHADOW_START_HEIGHT,
                PARAMETER_SINGLE_ANGLE_REVERSE,
                PARAMETER_SINGLE_ANGLE_FORWARD,
                PARAMETER_SINGLE_START_DATE,
                PARAMETER_SINGLE_STOP_DAYS,
	#endif

	// for internal usage
	PARAMETER_ELEVATION_DUTY_CYCLE,
	PARAMETER_AZIMUTH_DUTY_CYCLE,
	PARAMETER_MOVE_DISTANCE,
	PARAMETER_STEP_SIZE,					// for Move Sequence menu
	PARAMETER_X_COORD,						// for Move Sequence menu, Operation
	PARAMETER_Y_COORD,						// for Move Sequence menu, Operation

	PARAMETER_LED_STATE,

	// RTCC
	PARAMETER_RTCC_SECONDS,
	PARAMETER_RTCC_MINUTES,
	PARAMETER_RTCC_HOURS,
	PARAMETER_RTCC_DAY,
	PARAMETER_RTCC_DATE,
	PARAMETER_RTCC_MONTH,
	PARAMETER_RTCC_YEAR,

	PARAMETER_LAST

};

// **********************************************
//			Function Prototypes
// **********************************************
BOOL UpdateParameter(enum tagParameter eParameterToUpdate, char *ptrValueBuffer);

#ifdef USE_REMOTE_COMMANDS
	BOOL ReturnParameter(enum tagParameter eParameterToReturn, char *ptrReturnBuffer);
#endif



// end of UpdateParameter.h
