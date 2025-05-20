// *************************************************************************************************
//										C o o r d T r a n s l a t e . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Coordinate Translation function declarations
//
// *************************************************************************************************

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"				// gsf standard #defines
//#include "init.h"				// port definitions and initialization state
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions

//#include "DS3232.h"				// RTCC register level
//#include "RTCC.h"				// RTCC formatting

//#include "SunPosition.h"		// Sun Position Calculations

//#include "MotionProfile.h"		// degree to tick conversions
#include "MotionPhaseFSM.h"
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionLimits.h"
#include "MoveSequenceFSM.h"	// SPA_MOVE_INFO definition

//#include "SerialDisplay.h"	// display functions for menus
//#include "StrConversions.h"		// ASCII string <==> numeric conversions
#include "CoordTranslate.h"		// coordinate translation functions

//#include "Stubs.h"

#ifdef DEFINE_GLOBALS
	#error "DEFINE_GLOBALS not expected here"
#endif

// Convert from SPA orientation to Local orientation

// Output from the SPA is
//		Theta_AZ, the azimuth angle referenced from 180° (due south)
//		Theta_EL, elevation angle referenced from 0° (horizon)

// Theta_AZ is converted to AZ_MSI ticks relative to the center position, due south.
//		Positive values are to the west/right, and negative values are to the east/left.


// Local orientation is centered about:
//		Local Azimuth 0.0 degrees corresponds to SPA 180 degrees (due south) (is this correct? the diagrams below place south at 0!)
//		Local Elevation +45.0 corresponds to SPA + 45.0 degrees (45 degrees above horizon)

// we also need to account for physical offset from 0.0, 0.0, as set in system parameters


// SPA azimuth coordinate system
//
//					  N
//				+180    -180
//     +135						-135
//
// W +90								-90 E
//
//					  0
//					  S


// SPA azimuth coordinate system
//
//					  N
//				 +360   0
//     +315						45
//
// W +270								90 E
//
//					 180
//					  S


// Local coordinate system, based on facing the sun (northern hemisphere

//					  N
//				+180    -180
//     +135						-135
//
// W +90								-90 E
//  right, forward			left, reverse
//					  0
//					  S


void ConvertSPAtoLocalOrientation(PTR_ORIENTATION ptrOrientation)
{

	// <sek> SmartTrak had removed SPA to local conversion for Single Polar Axis implementation
	// Backtracking documentation shows movement from -45 degrees to +45 degrees with 0 in the middle, so I DO this this is what they want.
	//		NOTE: removing this conversion will will break 'Find End Points' and possibly other things..
        #if defined(USE_SINGLE_POLAR_AXIS)
		{
			 if(ptrOrientation->fElevation < 0)
                        {
                            ptrOrientation->fAzimuth = 0.0 ;
                        }
                        else
                        {
                            ptrOrientation->fAzimuth = ptrOrientation->fModuleTiltAngleDegrees ;
                        }
                }
	
        #else
		{
			ptrOrientation->fAzimuth -= 180.0;									// change center
			//ptrOrientation->fAzimuth -= ptrRAM_SystemParameters->fAZ_Offset;	// account for offset from center
		}
	#endif

	//ptrOrientation->fElevation -= ptrRAM_SystemParameters->fEL_Offset;	// account for offset from center


}


void ConvertSPAMoveDegreesToTicks(PTR_SPA_MOVE_INFO ptrSPAMove, PTR_ORIENTATION const ptrCurrentLocalOrientation)
{

	// ==>> should this be based on (newSPA - prevSPA) or (newSPA - currentOrientation) ?
	// ==>> this needs to account for AZ, EL offsets

	//*********************************
	//	Convert Ticks to Degrees
	//*********************************
	// convert previous and destination orientations from degrees to ticks
	ptrSPAMove->lPrevAzimuthTicks = ConvertDegreesToMSITicks(ptrSPAMove->fPrevAzimuth, AXIS_AZIMUTH);
	ptrSPAMove->lPrevElevationTicks = ConvertDegreesToMSITicks(ptrSPAMove->fPrevElevation, AXIS_ELEVATION);

	ptrSPAMove->lNewAzimuthTicks = ConvertDegreesToMSITicks(ptrSPAMove->fNewAzimuth, AXIS_AZIMUTH);
	ptrSPAMove->lNewElevationTicks = ConvertDegreesToMSITicks(ptrSPAMove->fNewElevation, AXIS_ELEVATION);


	//*********************************
	//			AZIMUTH
	// Calculate Required Move and Direction
	//*********************************
#ifdef USE_AZIMUTH
	// calculate required move based on difference between destination (lNewXxxTicks) and current orientation (lXxxxPositionTicks)
	// NOTE: these calculations are overkill; they could be combined, but as implemented here they are clear and cover all possible cases.
	if(ptrSPAMove->lNewAzimuthTicks IS ptrCurrentLocalOrientation->lAzimuthPositionTicks)
	{
		// no need to move
		ptrSPAMove->eAzimuthDirection = PWM_DIR_STOPPED;
		ptrSPAMove->lAzimuthMoveTicks = 0;
	}
	else if((ptrSPAMove->lNewAzimuthTicks > 0) AND (ptrCurrentLocalOrientation->lAzimuthPositionTicks > 0))
	{
		// the destination (lNewXxxTicks) > 0 and current orientation (lXxxxPositionTicks) > 0, so we are to the right or above the 0 point
		// both positions are POSITIVE, and the sign of the resulting move distance will determine the direction of the move
		if (ptrCurrentLocalOrientation->lAzimuthPositionTicks < ptrSPAMove->lNewAzimuthTicks)
		{
			// move forward to new position
			ptrSPAMove->lAzimuthMoveTicks = ptrSPAMove->lNewAzimuthTicks - ptrCurrentLocalOrientation->lAzimuthPositionTicks;
			ptrSPAMove->eAzimuthDirection = PWM_DIR_REVERSE;
		}
		else
		{
			// move reverse to new position
			ptrSPAMove->lAzimuthMoveTicks = ptrCurrentLocalOrientation->lAzimuthPositionTicks - ptrSPAMove->lNewAzimuthTicks;
			ptrSPAMove->eAzimuthDirection = PWM_DIR_FORWARD;
		}
	}
	else if((ptrCurrentLocalOrientation->lAzimuthPositionTicks <= 0) AND (ptrSPAMove->lNewAzimuthTicks > 0))
	{
		// current orientation (lXxxxPositionTicks) is to the left/below the 0 point, and destination (lNewXxxTicks) is above/right of the 0 point
		// move forward to new position
		// note minus sign to reverse polarity of ptrCurrentLocalOrientation->lAzimuthPositionTicks
		ptrSPAMove->lAzimuthMoveTicks = ptrSPAMove->lNewAzimuthTicks - ptrCurrentLocalOrientation->lAzimuthPositionTicks;
		ptrSPAMove->eAzimuthDirection = PWM_DIR_REVERSE;
	}
	else if((ptrSPAMove->lNewAzimuthTicks <= 0) AND (ptrCurrentLocalOrientation->lAzimuthPositionTicks > 0))
	{
		// current orientation (lNewXxxTicks) is to the left/below the 0 point, and destination (lXxxxPositionTicks) is above/right of the 0 point
		// move reverse to new position
		// note minus sign to reverse polarity of ptrSPAMove->lNewAzimuthTicks
		ptrSPAMove->lAzimuthMoveTicks = ptrCurrentLocalOrientation->lAzimuthPositionTicks - ptrSPAMove->lNewAzimuthTicks;
		ptrSPAMove->eAzimuthDirection = PWM_DIR_FORWARD;
	}
	else if ((ptrSPAMove->lNewAzimuthTicks <= 0) AND (ptrCurrentLocalOrientation->lAzimuthPositionTicks <= 0))
	{
		// the destination (lNewXxxTicks) < 0 and current orientation (lXxxxPositionTicks) < 0, so we are to the left or below the 0 point
		// both positions are NEGATIVE, and the sign of the resulting move distance will determine the direction of the move
		if (ptrCurrentLocalOrientation->lAzimuthPositionTicks < ptrSPAMove->lNewAzimuthTicks)
		{
			// move forward to new position
			// note minus sign to reverse polarity of ptrSPAMove->lNewAzimuthTicks
			ptrSPAMove->lAzimuthMoveTicks = ptrSPAMove->lNewAzimuthTicks - ptrCurrentLocalOrientation->lAzimuthPositionTicks;
			ptrSPAMove->eAzimuthDirection = PWM_DIR_REVERSE;
		}
		else
		{
			// move reverse to new position
			// note minus sign to reverse polarity of ptrSPAMove->lNewAzimuthTicks
			ptrSPAMove->lAzimuthMoveTicks = ptrCurrentLocalOrientation->lAzimuthPositionTicks - ptrSPAMove->lNewAzimuthTicks;
			ptrSPAMove->eAzimuthDirection = PWM_DIR_FORWARD;
		}
		
	}
	else
#endif	// USE_AZIMUTH
	{
		// current orientation (lXxxxPositionTicks) and destination (lNewXxxTicks) are both 0
		// OR we are somehow missing a case here....
		ptrSPAMove->eAzimuthDirection = PWM_DIR_STOPPED;
		ptrSPAMove->lAzimuthMoveTicks = 0;
	}

	//*********************************
	//			ELEVATION
	// Calculate Required Move and Direction
	//*********************************
#ifdef USE_ELEVATION
	// calculate required move based on difference between destination (lNewXxxTicks) and current orientation (lXxxxPositionTicks)
	// NOTE: these calculations are overkill; they could be combined, but as implemented here they are clear and cover all possible cases.
	if(ptrSPAMove->lNewElevationTicks IS ptrCurrentLocalOrientation->lElevationPositionTicks)
	{
		// no need to move
		ptrSPAMove->eElevationDirection = PWM_DIR_STOPPED;
		ptrSPAMove->lElevationMoveTicks = 0;
	}
	else 	if((ptrSPAMove->lNewElevationTicks > 0) AND (ptrCurrentLocalOrientation->lElevationPositionTicks > 0))
	{
		// the destination (lNewXxxTicks) > 0 and current orientation (lXxxxPositionTicks) > 0, so we are to the right or above the 0 point
		// both positions are POSITIVE, and the sign of the resulting move distance will determine the direction of the move
		if (ptrCurrentLocalOrientation->lElevationPositionTicks < ptrSPAMove->lNewElevationTicks)
		{
			// move forward to new position
			ptrSPAMove->lElevationMoveTicks = ptrSPAMove->lNewElevationTicks - ptrCurrentLocalOrientation->lElevationPositionTicks;
			ptrSPAMove->eElevationDirection = PWM_DIR_FORWARD;
		}
		else
		{
			// move reverse to new position
			ptrSPAMove->lElevationMoveTicks = ptrCurrentLocalOrientation->lElevationPositionTicks - ptrSPAMove->lNewElevationTicks;
			ptrSPAMove->eElevationDirection = PWM_DIR_REVERSE;
		}
	}
	else if((ptrCurrentLocalOrientation->lElevationPositionTicks <= 0) AND (ptrSPAMove->lNewElevationTicks > 0))
	{
		// move forward to new position
		// note minus sign to reverse polarity of ptrCurrentLocalOrientation->lElevationPositionTicks
		ptrSPAMove->lElevationMoveTicks = ptrSPAMove->lNewElevationTicks - ptrCurrentLocalOrientation->lElevationPositionTicks;
		ptrSPAMove->eElevationDirection = PWM_DIR_FORWARD;
	}
	else if((ptrSPAMove->lNewElevationTicks <= 0) AND (ptrCurrentLocalOrientation->lElevationPositionTicks > 0))
	{
		// move reverse to new position
		// note minus sign to reverse polarity of ptrSPAMove->lNewElevationTicks
		ptrSPAMove->lElevationMoveTicks = ptrCurrentLocalOrientation->lElevationPositionTicks - ptrSPAMove->lNewElevationTicks;
		ptrSPAMove->eElevationDirection = PWM_DIR_REVERSE;
	}
	else if ((ptrSPAMove->lNewElevationTicks <= 0) AND (ptrCurrentLocalOrientation->lElevationPositionTicks <= 0))
	{
		if (ptrCurrentLocalOrientation->lElevationPositionTicks < ptrSPAMove->lNewElevationTicks)
		{
			// move forward to new position
			// note minus sign to reverse polarity of ptrSPAMove->lNewElevationTicks
			ptrSPAMove->lElevationMoveTicks = ptrSPAMove->lNewElevationTicks - ptrCurrentLocalOrientation->lElevationPositionTicks;
			ptrSPAMove->eElevationDirection = PWM_DIR_FORWARD;
		}
		else
		{
			// move reverse to new position
			// note minus sign to reverse polarity of ptrSPAMove->lNewElevationTicks
			ptrSPAMove->lElevationMoveTicks = ptrCurrentLocalOrientation->lElevationPositionTicks - ptrSPAMove->lNewElevationTicks;
			ptrSPAMove->eElevationDirection = PWM_DIR_REVERSE;
		}
	}
	else
#endif	// USE_ELEVATION
	{
		// current orientation (lXxxxPositionTicks) and destination (lNewXxxTicks) are both 0
		// we are somehow missing a case here....
		ptrSPAMove->eElevationDirection = PWM_DIR_STOPPED;
		ptrSPAMove->lElevationMoveTicks = 0;

	}
}

// <sek> 23 Sep 13 works OK? checked in debugger
float ConvertMSITicksToDegrees(INT32 lTicks, enum tagAxis eAxis)
{
	switch (eAxis)
	{
		case AXIS_AZIMUTH:
			return (((float)lTicks) / AZ_FLOAT_MSI_TICKS_PER_DEGREE);

		case AXIS_ELEVATION:
			return (((float)lTicks) / EL_FLOAT_MSI_TICKS_PER_DEGREE);

		case AXIS_NONE:
		default:
			return 0.0;
	}

}

// 17 Aug 13 <sek> this was returning UINT32
INT32 ConvertDegreesToMSITicks(float fDegrees, enum tagAxis eAxis)
{

	switch (eAxis)
	{
		case AXIS_AZIMUTH:
			return (INT32)((fDegrees * AZ_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET);

		case AXIS_ELEVATION:
			return (INT32)((fDegrees * EL_FLOAT_MSI_TICKS_PER_DEGREE) + CAST_FLOAT_ROUNDING_OFFSET);

		case AXIS_NONE:
		default:
			return (INT32) 0;

	}

}


// end of CoordTranslate.c

