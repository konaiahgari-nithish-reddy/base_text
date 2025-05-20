// *************************************************************************************************
//										U p d a t e P a r a m e t e r s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Persistent parameter update, used for menus and remote commands
//
// *************************************************************************************************

#include <GenericTypeDefs.h>	// includes stddef.h

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

//#define DEFINE_GLOBALS

#include "EventFlags.h"			// event flag definitions and globals
////#include "SerialPort.h"
#include "SerialDisplay.h"		// display functions for menus
#include "StrConversions.h"		// ASCII string <==> numeric conversions
#include "ftoa.h"

//#undef DEFINE_GLOBALS

////#include "I2CBus.h"
#include "DS3232.h"				// RTCC register level
#include "RTCC.h"				// RTCC formatting
#ifdef USE_PCA9554_IO			// enable in config.h ONLY if PCA9554 hardware is present
#include "PCA9554.h"		// SPI expander definitions
#endif	// USE_PCA9554_IO
#ifdef USE_MMA8452Q_INCLINOMETER
#include "mma845x.h"         // MMA845xQ macros
#include "Inclinometer.h"
#endif	//  USE_MMA8452Q_INCLINOMETER


#include "UpdateParameters.h"
//#define DEFINE_GLOBALS
#include "MenuFSM.h"			// for Closed Loop Menu and Move Sequence Menu globals
//#undef DEFINE_GLOBALS
//#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions
#include "Debounce.h"			// Input Switch debounce functions
#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM

#include "ButtonProcessingFSM.h"// Button and user input processing
#include"ftoa.h"

#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
#include "CoordTranslate.h"		// coordinate translation and formatting
////#include "Stubs.h"

#include <ctype.h>				// tolower()
#include <string.h>				// Microchip string functions

#ifdef DEFINE_GLOBALS
#error "DEFINE_GLOBALS not expected here"
#endif


//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagUpdateParameterErrors
{
	UPDATE_PARAM_ERROR_NONE = UPDATE_PARAM_ERROR_BASE,
	UPDATE_PARAM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	UPDATE_PARAM_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	UPDATE_PARAM_ERROR_INVALID_STATE,			// 3 not a valid state
	UPDATE_PARAM_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	UPDATE_PARAM_ERROR_INVALID_SELECTION,		// 5 not a valid selection
	UPDATE_PARAM_ERROR_UNKNOWN_COMMAND,			// 6 not a valid command
	UPDATE_PARAM_ERROR_INVALID_PARAMETER,		// 7 not a valid parameter selection
	UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER,	// 8 not a valid parameter selection
	UPDATE_PARAM_ERROR_BUFFER_OVERFLOW,			// 9 menu string too long
	UPDATE_PARAM_ERROR_RTCC_READ_ERROR,			// A RTCC read failure

	UPDATE_PARAM_UNPROCESSED_EVENT = UPDATE_PARAM_ERROR_BASE + 0x0F
};


//-------------------------------------------------------------------------------------------------------
// Forward References - File Local function Definitions
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// File Local - Global Variables
//-------------------------------------------------------------------------------------------------------


//-----------------------------------------------
// keep track of parameter to update

//PRIVATE_INIT enum tagParameter eParameter = PARAMETER_NONE;


// *****************************************************************************
//					U p d a t e P a r a m e t e r ( )
// *****************************************************************************

// This is going to get quite large, so it needs to be converted to a TABLE
BOOL UpdateParameter(enum tagParameter eParameterToUpdate, char *ptrValueBuffer)
{

	UINT8 bParameterValue; int intparam;
	//	UINT16 nParameterValue;
	float fParameterValue;
	BOOL bValueValid = TRUE;

	ClearDisplayStr();											// initialize output buffer for possible serial output string

	switch(eParameterToUpdate)
	{

	// ***************************************
	//			Location
	// ***************************************
	case PARAMETER_LATITUDE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= MIN_LATITUDE) && (fParameterValue <= MAX_LATITUDE))
		{
			ptrRAM_SystemParameters->fLatitude = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_LONGITUDE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= MIN_LONGITUDE) && (fParameterValue <= MAX_LONGITUDE))
		{
			ptrRAM_SystemParameters->fLongitude = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_ALTITUDE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= MIN_ALTITUDE) && (fParameterValue <= MAX_ALTITUDE))
		{
			ptrRAM_SystemParameters->fAltitude = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_REFRACTION:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= MIN_REFRACTION) && (fParameterValue <= MAX_REFRACTION))
		{
			ptrRAM_SystemParameters->fRefraction = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_TIMEZONE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= MIN_TIMEZONE) && (fParameterValue <= MAX_TIMEZONE))
		{
			ptrRAM_SystemParameters->fTimeZone = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_TRACKING_MODE:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to BYTE

		// bounds check
		if ((bParameterValue >= MODE_MANUAL) && (bParameterValue <= MODE_WIND_STOW))
		{
			ptrRAM_SystemParameters->ucTracking_Mode = bParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

		// ***************************************
		//			Open Loop Menu
		// ***************************************
	case PARAMETER_ELEVATION_DUTY_CYCLE:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UIN16

		if (PWM_SetDutyCycle(MOTOR_ELEVATION, bParameterValue) IS_NOT TRUE)	// sets pgbPWMDutyCycle
		{
			bValueValid = FALSE;							// not a valid entry
		}
#ifdef USE_FEEDBACK_SIMULATOR
		else
		{
			PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_ELEVATION, bParameterValue);		// set MotionProfileSpeedAndPWM[] index for simulator
			MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);		// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
		}
#endif
		break;

	case PARAMETER_AZIMUTH_DUTY_CYCLE:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UIN16

		if (PWM_SetDutyCycle(MOTOR_AZIMUTH, bParameterValue) IS_NOT TRUE)	// sets pgbPWMDutyCycle
		{
			bValueValid = FALSE;							// not a valid entry
		}
#ifdef USE_FEEDBACK_SIMULATOR
		else
		{
			PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_AZIMUTH, bParameterValue);		// set MotionProfileSpeedAndPWM[] index for simulator
			MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);		// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]

		}
#endif
		break;


		// ***************************************
		//	Closed Loop Menu
		// ***************************************
	case PARAMETER_MOVE_DISTANCE:							// for closed loop menu
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= 90.0))
		{
			fgfCLMoveDistance = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

		// ***************************************
		//			Move Sequence Menu
		// ***************************************
	case PARAMETER_STEP_SIZE:								// for Move Sequence menu
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= 90.0))
		{
			fgfMSStepSize = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;


	case PARAMETER_X_COORD:									// for Move Sequence menu, Operation
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= -90.0) && (fParameterValue <= 90.0))
		{
			fgfMoveToXCoord = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;


	case PARAMETER_Y_COORD:									// for Move Sequence menu, Operation
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= 90.0))
		{
			fgfMoveToYCoord = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;


	case PARAMETER_LED_STATE:
#ifdef USE_PCA9554_IO									// enable in config.h ONLY if PCA9554 hardware is present
		bParameterValue = (UINT8)xtoi(ptrValueBuffer);		// convert input string to UINT8
		SetLEDState(bParameterValue);						// write to I/O expander
#endif	// USE_PCA9554_IO
		break;

		// **************************************
		//			Azimuth
		// **************************************
#ifdef USE_AZIMUTH
	case PARAMETER_AZ_OFFSET:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fAZ_Offset= fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_SOFT_LIMIT_REV:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue < AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_SOFT_LIMIT_FWD:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue > AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fAZ_SoftLimit_Forward = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_DEAD_BAND:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= 2.0))
		{
			ptrRAM_SystemParameters->fAZ_DeadBand = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_NIGHT_STOW_THRESHOLD:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= -10.0) && (fParameterValue <= 10.0))
		{
			ptrRAM_SystemParameters->fAZ_NightStowThreshold = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_NIGHT_STOW_POS:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fAZ_NightStowPosition = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_AZ_WIND_STOW_POS:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fAZ_WindStowPosition = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;
#endif

		// **************************************
		//			Elevation
		// **************************************
		// elevation values are always required to trigger NIGHT_STOW
	case PARAMETER_EL_OFFSET:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= EL_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= EL_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fEL_Offset= fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_SOFT_LIMIT_REV:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= EL_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue < EL_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fEL_SoftLimit_Reverse = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_SOFT_LIMIT_FWD:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue > EL_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= EL_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fEL_SoftLimit_Forward = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_DEAD_BAND:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= 2.0))
		{
			ptrRAM_SystemParameters->fEL_DeadBand = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_NIGHT_STOW_THRESHOLD:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= -10.0) && (fParameterValue <= 10.0))
		{
			ptrRAM_SystemParameters->fEL_NightStowThreshold = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_NIGHT_STOW_POS:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= EL_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue < EL_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fEL_NightStowPosition = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_EL_WIND_STOW_POS:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= EL_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= EL_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fEL_WindStowPosition = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

		// **************************************
		//			BackTracking
		// **************************************

#ifdef USE_BACKTRACKING
	case PARAMETER_BACKTRACKING_ENABLED:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to BYTE

		// bounds check
		if ((bParameterValue IS 0) || (bParameterValue IS 1))
		{
			ptrRAM_SystemParameters->bBacktrackingEnabled = bParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_PANEL_SHADOW_START_ANGLE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_SUN_SHADOW_START_ANGLE:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0) && (fParameterValue <= 90))
		{
			ptrRAM_SystemParameters->fSunShadowStartAngleDegrees = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_SUN_SHADOW_START_HEIGHT:
		fParameterValue = atof(ptrValueBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0.0) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fSunShadowStartHeight = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;
	case PARAMETER_SINGLE_ANGLE_REVERSE:
		fParameterValue = atof(pgcInputBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= AZ_SOFT_LIMIT_DEGREES_REVERSE) && (fParameterValue <= 0))
		{
			ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_SINGLE_ANGLE_FORWARD:
		fParameterValue = atof(pgcInputBuffer);				// convert input string to float

		// bounds check
		if ((fParameterValue >= 0) && (fParameterValue <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
		{
			ptrRAM_SystemParameters->fSingle_SoftLimit_Forward = fParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

#endif	// USE_BACKTRACKING

	case PARAMETER_SINGLE_START_DATE:
		// bounds check
		intparam = atoi(pgcInputBuffer);
		if ((intparam >= 0))
		{
			ptrRAM_SystemParameters->fSingle_start_date = intparam;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;
	case PARAMETER_SINGLE_STOP_DAYS:
		// bounds check
		bParameterValue = atoi(ptrValueBuffer);
		if ((bParameterValue >= 0))
		{
			ptrRAM_SystemParameters->fSingle_stop_days = bParameterValue;
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;
		// **************************************
		//				RTCC
		// **************************************
	case PARAMETER_RTCC_SECONDS:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8

		if ((bParameterValue >= DS3232_MIN_SECONDS) && (bParameterValue <= DS3232_MAX_SECONDS))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_SECONDS, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_MINUTES:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_MINUTES) && (bParameterValue <= DS3232_MAX_MINUTES))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_MINUTES, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_HOURS:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_HOURS_24) && (bParameterValue <= DS3232_MAX_HOURS_24))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_HOURS, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_DAY:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_DAY) && (bParameterValue <= DS3232_MAX_DAY))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_DAY, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_DATE:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_DATE) && (bParameterValue <= DS3232_MAX_DATE))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_DATE, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_MONTH:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_MONTH) && (bParameterValue <= DS3232_MAX_MONTH))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_MONTH, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_YEAR:
		bParameterValue = (UINT8)atoi(ptrValueBuffer);		// convert input string to UINT8
		if ((bParameterValue >= DS3232_MIN_YEAR) && (bParameterValue <= DS3232_MAX_YEAR))
		{
//			bParameterValue = BYTEtoBCD(bParameterValue);		// convert UINT8 to BCD
			if (WriteRTCCRegister(DS3232_REG_YEAR, bParameterValue) IS_FALSE)
			{
				DisplayMessage("Write RTCC Register Failed", WAIT_FOR_DISPLAY);
			}
		}
		else
		{
			RuntimeError(UPDATE_PARAM_ERROR_OUT_OF_RANGE_PARAMETER);
			bValueValid = FALSE;							// not a valid entry
		}
		break;


		// **************************************
		//			Unknown Parameter
		// **************************************

	case PARAMETER_NONE:
	default:
		RuntimeError(UPDATE_PARAM_ERROR_INVALID_PARAMETER);
		bValueValid = FALSE;								// not a valid entry
		break;
	}

	return(bValueValid);

}

// *****************************************************************************
//					R e t u r n  P a r a m e t e r ( )
// *****************************************************************************

BOOL ReturnParameter(enum tagParameter eParameterToReturn, char *ptrReturnBuffer)
{
	UINT8 bParameterValue;
	//	UINT16 nParameterValue;
	//	float fParameterValue;
	BOOL bValueValid = TRUE;
	int ftoaStatus;

	switch(eParameterToReturn)
	{

	// ***************************************
	//			Location
	// ***************************************
	case PARAMETER_LATITUDE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fLatitude, &ftoaStatus));
		break;

	case PARAMETER_LONGITUDE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fLongitude, &ftoaStatus));
		break;

	case PARAMETER_ALTITUDE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAltitude, &ftoaStatus));
		break;

	case PARAMETER_REFRACTION:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fRefraction, &ftoaStatus));
		break;

	case PARAMETER_TIMEZONE:
		// NOTE: TimeZone is a float because it can be fractional; Hyderabad is GMT - 5.5 hours (?)
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fTimeZone, &ftoaStatus));
		break;

	case PARAMETER_TRACKING_MODE:
		BYTEtoASCIIstr(ptrRAM_SystemParameters->ucTracking_Mode, ptrReturnBuffer);
		break;

		// ***************************************
		//			Open Loop Menu
		// ***************************************
	case PARAMETER_ELEVATION_DUTY_CYCLE:
		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_ELEVATION], ptrReturnBuffer);
		break;

	case PARAMETER_AZIMUTH_DUTY_CYCLE:
		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_AZIMUTH], ptrReturnBuffer);
		break;


		// ***************************************
		//			Closed Loop Menu
		// ***************************************
	case PARAMETER_MOVE_DISTANCE:							// for closed loop menu
		strcpy(ptrReturnBuffer, ftoa(fgfCLMoveDistance, &ftoaStatus));
		break;

		// ***************************************
		//			Move Sequence Menu
		// ***************************************
	case PARAMETER_STEP_SIZE:								// for Move Sequence menu
		strcpy(ptrReturnBuffer, ftoa(fgfMSStepSize, &ftoaStatus));
		break;


	case PARAMETER_X_COORD:									// for Move Sequence menu, Operation
		strcpy(ptrReturnBuffer, ftoa(fgfMoveToXCoord, &ftoaStatus));
		break;


	case PARAMETER_Y_COORD:									// for Move Sequence menu, Operation
		strcpy(ptrReturnBuffer, ftoa(fgfMoveToYCoord, &ftoaStatus));
		break;


	case PARAMETER_LED_STATE:
#ifdef USE_PCA9554_IO									// enable in config.h ONLY if PCA9554 hardware is present
		BYTEtoHexASCIIstr(GetLEDState(), ptrReturnBuffer);
#endif	// USE_PCA9554_IO
		break;

		// **************************************
		//			Azimuth
		// **************************************
#ifdef USE_AZIMUTH
	case PARAMETER_AZ_OFFSET:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_Offset, &ftoaStatus));
		break;

	case PARAMETER_AZ_SOFT_LIMIT_REV:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse, &ftoaStatus));
		break;

	case PARAMETER_AZ_SOFT_LIMIT_FWD:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward, &ftoaStatus));
		break;

	case PARAMETER_AZ_DEAD_BAND:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_DeadBand, &ftoaStatus));
		break;

	case PARAMETER_AZ_NIGHT_STOW_THRESHOLD:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_NightStowThreshold, &ftoaStatus));
		break;

	case PARAMETER_AZ_NIGHT_STOW_POS:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_NightStowPosition, &ftoaStatus));
		break;

	case PARAMETER_AZ_WIND_STOW_POS:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fAZ_WindStowPosition, &ftoaStatus));
		break;
#endif

// **************************************
		//			Elevation
		// **************************************
		// elevation values are always required to trigger Night Stow
	case PARAMETER_EL_OFFSET:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_Offset, &ftoaStatus));
		break;

	case PARAMETER_EL_SOFT_LIMIT_REV:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse, &ftoaStatus));
		break;

	case PARAMETER_EL_SOFT_LIMIT_FWD:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_SoftLimit_Forward, &ftoaStatus));
		break;

	case PARAMETER_EL_DEAD_BAND:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_DeadBand, &ftoaStatus));
		break;

	case PARAMETER_EL_NIGHT_STOW_THRESHOLD:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_NightStowThreshold, &ftoaStatus));
		break;

	case PARAMETER_EL_NIGHT_STOW_POS:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_NightStowPosition, &ftoaStatus));
		break;

	case PARAMETER_EL_WIND_STOW_POS:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fEL_WindStowPosition, &ftoaStatus));
		break;

		// **************************************
		//			BackTracking
		// **************************************

#ifdef USE_BACKTRACKING
	case PARAMETER_BACKTRACKING_ENABLED:
		BYTEtoASCIIstr(ptrRAM_SystemParameters->bBacktrackingEnabled, ptrReturnBuffer);
		break;

	case PARAMETER_PANEL_SHADOW_START_ANGLE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees, &ftoaStatus));
		break;

	case PARAMETER_SUN_SHADOW_START_ANGLE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fSunShadowStartAngleDegrees, &ftoaStatus));
		break;

	case PARAMETER_SUN_SHADOW_START_HEIGHT:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fSunShadowStartHeight, &ftoaStatus));
		break;
	case PARAMETER_SINGLE_ANGLE_REVERSE:
		strcpy(ptrReturnBuffer, ftoa(ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse, &ftoaStatus));
		break;
	case PARAMETER_SINGLE_START_DATE:
		INT32UtoASCIIstr(ptrRAM_SystemParameters->fSingle_start_date,INT32U_WIDTH,ptrReturnBuffer);
		break;
	case PARAMETER_SINGLE_STOP_DAYS:
		BCDBytetoASCIIstr(ptrRAM_SystemParameters->fSingle_stop_days, ptrReturnBuffer);
		break;
#endif	// USE_BACKTRACKING

// **************************************
//				RTCC
// **************************************
	case PARAMETER_RTCC_SECONDS:
		if (ReadRTCCRegister(DS3232_REG_SECONDS, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_MINUTES:
		if (ReadRTCCRegister(DS3232_REG_MINUTES, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_HOURS:
		if (ReadRTCCRegister(DS3232_REG_HOURS, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_DAY:
		if (ReadRTCCRegister(DS3232_REG_DAY, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_DATE:
		if (ReadRTCCRegister(DS3232_REG_DATE, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_MONTH:
		if (ReadRTCCRegister(DS3232_REG_MONTH, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

	case PARAMETER_RTCC_YEAR:
		if (ReadRTCCRegister(DS3232_REG_YEAR, &bParameterValue) IS_TRUE)
		{
			BCDBytetoASCIIstr(bParameterValue, ptrReturnBuffer);
		}
		else
		{
			strcpy("00", ptrReturnBuffer);
			RuntimeError(UPDATE_PARAM_ERROR_RTCC_READ_ERROR);
			bValueValid = FALSE;							// not a valid entry
		}
		break;

		// **************************************
		//			Unknown Parameter
		// **************************************

	case PARAMETER_NONE:
	default:
		RuntimeError(UPDATE_PARAM_ERROR_INVALID_PARAMETER);
		bValueValid = FALSE;								// not a valid entry
		break;
	}

	return(bValueValid);

}


// end of UpdateParameters.c
