// *************************************************************************************************
//										S u n P o s i t i o n . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Sun Position Algorithm functions

//
// *************************************************************************************************

// this algorithm is based entirely on documentation provided by SmartTrak Solar Systems Pvt, Hyderabad, AP, India
// see Smarttrak_SunModel_20130306.pdf

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14
#include <legacy\int_3xx_4xx_legacy.h>	// required for various interrupt handlers

#include <math.h>				// trig functions

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions

#include "I2CBus.h"
#include "DS3232.h"				// Real Time Clock

#include "SunPosition.h"		// Sun Position Calculations

//#include "HardwareProfile.h"
//#include "SerialPort.h"
#include "SerialDisplay.h"		// display functions for menus


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


// *************************************************************************************************
//
// *************************************************************************************************

/* cumulative number of days prior to beginning of month */
ARRAY static const UINT16 month_days[2][13] = {
										{ 0,   0,  31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334 },
										{ 0,   0,  31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335 }	// leap year
									 };

static const float fRadiansToDegrees = 57.295779513;		/* converts from radians to degrees */
static const float fDegreesToRadians = 0.0174532925;		/* converts from degrees to radians */
static const float fDegreesPerOrbit = 360.0;
static const float fDegreesPerRevolution = 360.0;
static const float fDaysPerYear = 365.25;
static const float fHoursPerDay = 24.0;
static const float fMinutesPerHour = 60.0;



// this structure contains all of the intermediate calculation results, and the output
typedef struct
{
	UINT16	nDayNumber;
	float	fHours;
	float	fDayOfYearDegrees;
	float	fDayOfYearRadians;
	float	fDeclinationAngleDegrees;
	float	fTimeCorrection;
	float	fHourAngleDegrees;
	float	fCosExoZenithAngle;
	float	fExoZenithAngleDegrees;
	float	fRefractionCorrectionDegrees;
	float	fZenithAngleDegrees;
	float	fCosExoAzimuthAngle;
	float	fExoAzimuthAngleDegrees;
	float	fAzimuthAngleDegrees;
	#ifdef USE_BACKTRACKING
		float	fSunElevationAngleDegrees;
		float   fAzimuthTiltAngleDegrees;
		float   fModuleTiltAngleDegrees;
		float   fPanelShadowStartAngleDegrees;
		float   fSunShadowStartHeight;
		float   fSunShadowStartAngleDegrees;
		float	fBackTrack;
	#endif	// USE_SINGLE_POLAR_AXIS

} POSITION_INFO, *PTR_POSITION_INFO;

FILE_GLOBAL POSITION_INFO fgPositionInfo;

// *************************************************************************************************
//								Sun Position Calculations
// *************************************************************************************************

// Get current Julian Date (Jan 1 = 1, Dec 31 = 365, except in leap year Dec 31 = 366

UINT8 GetDayOfYear(PTR_RTCC_DATE_TIME ptrCurrentDateTime)
{

	// determine day of year, without considering leapyear
	fgPositionInfo.nDayNumber = (UINT16)ptrCurrentDateTime->cDate + month_days[0][ptrCurrentDateTime->cMonth];

	// adjust for leap year
	//			divisible by 4					AND (		not a xx00 year					 OR      every 400 years					) AND   past end of February
	if ( ((ptrCurrentDateTime->nYear % 4) IS 0) && ( ((ptrCurrentDateTime->cDay % 100) != 0) || ((ptrCurrentDateTime->cDay % 400) IS 0) ) && (ptrCurrentDateTime->cMonth > 2) )
		fgPositionInfo.nDayNumber += 1;

	return(fgPositionInfo.nDayNumber);

}


// get current year degrees
// 360 degree orbit, 365.25 days
// hours include minutes as fractional hour
// g = (360/365.25) * (Julian Date + hour/24)
// note -1, so that on day 1 we get an angle of 0

void GetYearDegrees(PTR_RTCC_DATE_TIME ptrCurrentDateTime)
{
	// calculate hours into the day, including minutes as fractional hour
	fgPositionInfo.fHours = (float)ptrCurrentDateTime->cHours + ((float)ptrCurrentDateTime->cMinutes / fMinutesPerHour);

	fgPositionInfo.fDayOfYearDegrees = (fDegreesPerOrbit / fDaysPerYear) * ((float)(fgPositionInfo.nDayNumber - 1) + (fgPositionInfo.fHours / fHoursPerDay));

}

// Calculate Declination Angle
//Declination Angle
//D = 0.396372 - (22.91327 * cos(g)) + (4.02543 * sin(g)) - (0.387205 * cos(2*g)) + (0.051967 * sin(2*g)) - (0.154527 * cos(3*g)) + (0.084798*sin(3*g))
float CalculateDeclinationAngle()
{
	// calculate Year Degrees in Radian
	fgPositionInfo.fDayOfYearRadians = fgPositionInfo.fDayOfYearDegrees * fDegreesToRadians;

	/* from older document, Sep 2012
	fgPositionInfo.fDeclinationAngle = 0.396372
			- (22.91327 * cosf(fgPositionInfo.fDayOfYearRadians)) + (4.02543 * sinf(fgPositionInfo.fDayOfYearRadians))
			- (0.387205 * cosf(2 * fgPositionInfo.fDayOfYearRadians)) + (0.051967 * sinf(2 * fgPositionInfo.fDayOfYearRadians))
			- (0.154527 * cosf(3 * fgPositionInfo.fDayOfYearRadians)) + (0.084798 * sinf(3 * fgPositionInfo.fDayOfYearRadians))
	 */

	//lint -e834	error 834: (Info -- Operator '-' followed by operator '+' is confusing.  Use parentheses.)
	// from newer document, Mar 2013. Some of these numbers will introduce errors, because they are too close to 0.0
	fgPositionInfo.fDeclinationAngleDegrees = 57.29578 *
		(
			0.006918
			- (0.399912 * cosf(fgPositionInfo.fDayOfYearRadians)) + (0.070257 * sinf(fgPositionInfo.fDayOfYearRadians))
			- (0.006758 * cosf(2 * fgPositionInfo.fDayOfYearRadians)) + (0.000907 * sinf(2 * fgPositionInfo.fDayOfYearRadians))
			- (0.002697 * cosf(3 * fgPositionInfo.fDayOfYearRadians)) + (0.00148 * sinf(3 * fgPositionInfo.fDayOfYearRadians))
		);
	//lint +e834

	return(fgPositionInfo.fDeclinationAngleDegrees);

}

// Calculate Time Correction
//	TC = 0.004297+0.107029*cos(g)-1.837877*sin(g)-0.837378*cos(2*g)- -2.340475*sin(2*g)

float CalculateTimeCorrection(PTR_RTCC_DATE_TIME ptrCurrentDateTime)
{
	/* from older document, Sep 2012
	fgPositionInfo.fTimeCorrection = 0.004297 +
		(0.107029*cosf(fgPositionInfo.fDayOfYearRadians)) - (1.837877 * sinf(fgPositionInfo.fDayOfYearRadians))
		- (0.837378 * cosf(2 * fgPositionInfo.fDayOfYearRadians)) - (2.340475 * sinf(2 * fgPositionInfo.fDayOfYearRadians);
	*/

	//lint -e834	error 834: (Info -- Operator '-' followed by operator '+' is confusing.  Use parentheses.)
	// from newer document, Mar 2013. Some of these numbers will introduce errors, because they are too close to 0.0
	fgPositionInfo.fTimeCorrection = (0.000075 + (0.001868 * cosf(fgPositionInfo.fDayOfYearRadians)) - (0.032077 * sinf(fgPositionInfo.fDayOfYearRadians))
		- (0.014615 * cosf(2 * fgPositionInfo.fDayOfYearRadians)) - (0.040849 * sinf(2 * fgPositionInfo.fDayOfYearRadians)))
			* (229.18 / 60.0);
	//lint +e834

	return(fgPositionInfo.fTimeCorrection);

}

// Calculate Hour Angle

float CalculateHourAngle(PTR_RTCC_DATE_TIME ptrCurrentDateTime)
{
	// Theta_HR = (Hour + (Minute/60) - TimeZone + Time Correction) * (360/24) + Latitude - 180;

	//lint -e834	error 834: (Info -- Operator '-' followed by operator '+' is confusing.  Use parentheses.)
	// NOTE: fgPositionInfo.fHours already includes minutes as fractional hours
	fgPositionInfo.fHourAngleDegrees = ((fgPositionInfo.fHours - ptrRAM_SystemParameters->fTimeZone + fgPositionInfo.fTimeCorrection) * (fDegreesPerRevolution/fHoursPerDay))
											+ ptrRAM_SystemParameters->fLongitude - 180;
	//lint +e834

	return(fgPositionInfo.fHourAngleDegrees);
}


// Calculate Exoatomspheric Zenith Angle (For Elevation)
float CalculateExoZenithAngle()
{
	// cos(Theta_EZ) = (sin(Theta_Declination) * sin(Theta_Latitude)) + (cos(Theta_Declination) * cos(Theta_Latitude) * cos(Theta_HR))
	fgPositionInfo.fCosExoZenithAngle =
		(
			(sinf(fgPositionInfo.fDeclinationAngleDegrees * fDegreesToRadians) * sinf(ptrRAM_SystemParameters->fLatitude * fDegreesToRadians))
			+ (cosf(fgPositionInfo.fDeclinationAngleDegrees * fDegreesToRadians) * cosf(ptrRAM_SystemParameters->fLatitude * fDegreesToRadians) * cosf(fgPositionInfo.fHourAngleDegrees  * fDegreesToRadians))
		);

	if (fgPositionInfo.fCosExoZenithAngle >= 1.0)
	{
		fgPositionInfo.fExoZenithAngleDegrees = 0.0;
	}
	else if (fgPositionInfo.fCosExoZenithAngle <= -1.0)
	{
		fgPositionInfo.fExoZenithAngleDegrees = 180.0;
	}
	else
	{
		fgPositionInfo.fExoZenithAngleDegrees = acosf(fgPositionInfo.fCosExoZenithAngle) * fRadiansToDegrees;
	}

	return(fgPositionInfo.fExoZenithAngleDegrees);
}


float CalculateRefractionCorrection()
{
	// not currently implemented
	fgPositionInfo.fRefractionCorrectionDegrees = 0.0;

	return(fgPositionInfo.fRefractionCorrectionDegrees);
}

 // Calculate Solar Azimuth
float CalculateSolarZenithAngle()
{

	fgPositionInfo.fZenithAngleDegrees = fgPositionInfo.fExoZenithAngleDegrees - fgPositionInfo.fRefractionCorrectionDegrees;

	return(fgPositionInfo.fZenithAngleDegrees);

}

// Calculate Exoatomspheric Azimuth Angle (For Azimuth)
float CalculateExoAzimuthAngle()
{
	// cos(Theta_EA) = (cos(Theta_EZ) * sin(Theta_Latitude)) - (sin(Theta_Declination)) / ((sin(Theta_EZ) * cos(Theta_Latitude))
	fgPositionInfo.fCosExoAzimuthAngle =
			( (cosf(fgPositionInfo.fExoZenithAngleDegrees * fDegreesToRadians) * sinf(ptrRAM_SystemParameters->fLatitude * fDegreesToRadians)) - sinf(fgPositionInfo.fDeclinationAngleDegrees * fDegreesToRadians) )
			/ ( (sinf(fgPositionInfo.fExoZenithAngleDegrees * fDegreesToRadians) * cosf(ptrRAM_SystemParameters->fLatitude * fDegreesToRadians)) );

	if (fgPositionInfo.fCosExoAzimuthAngle >= 1.0)
	{
		fgPositionInfo.fExoAzimuthAngleDegrees = 0.0;
	}
	else if (fgPositionInfo.fCosExoAzimuthAngle <= -1.0)
	{
		fgPositionInfo.fExoAzimuthAngleDegrees = 180.0;
	}
	else
	{
		fgPositionInfo.fExoAzimuthAngleDegrees = acosf(fgPositionInfo.fCosExoAzimuthAngle) * fRadiansToDegrees;
	}

	return(fgPositionInfo.fExoAzimuthAngleDegrees);
}

 // Calculate Solar Azimuth
float CalculateSolarAzimuthAngle()
{

	if (fabs(fgPositionInfo.fHourAngleDegrees) IS_NOT 0.0)		// check for divide by 0
	{
		fgPositionInfo.fAzimuthAngleDegrees = 180.0 + ((fgPositionInfo.fHourAngleDegrees / fabs(fgPositionInfo.fHourAngleDegrees)) * fgPositionInfo.fExoAzimuthAngleDegrees);
	}
	// else just leave in place the previous calculation

	return(fgPositionInfo.fAzimuthAngleDegrees);
}


// *************************************************************************************************
//							Single Polar Axis Backtracking Calculations
// *************************************************************************************************

#ifdef USE_BACKTRACKING
	// this does not appear to be used for anything
	float CalculateSunElevationAngle()
	{
		// EL = 90 â€“ Solar Zenith Angle
		fgPositionInfo.fSunElevationAngleDegrees = 90.0 - fgPositionInfo.fZenithAngleDegrees;

		return(fgPositionInfo.fSunElevationAngleDegrees);
	}


	float CalculateAzimuthTilt()
	{
		//AZ Tilt = 180 â€“ Azimuth Angle
		// ==> this is the same as converting from SPA to local coordinates
		fgPositionInfo.fAzimuthTiltAngleDegrees = 180.0 - fgPositionInfo.fAzimuthAngleDegrees;

		return(fgPositionInfo.fAzimuthTiltAngleDegrees);
	}


	float CalculateModuleTilt()
	{
		float fTiltARadians, fTiltBRadians, fTiltModule;
		float fTA = 0.0;			// Northern Hemisphere, TA = 0

		// A= [Sin (ZN) * Cos (AZ tilt) * Sin (TA)] + [Cos (ZN) * Cos (TA)]
		fTiltARadians = ( sinf( fgPositionInfo.fZenithAngleDegrees * fDegreesToRadians) * cosf(fgPositionInfo.fAzimuthTiltAngleDegrees * fDegreesToRadians) * sinf(fTA) ) + ( cosf(fgPositionInfo.fZenithAngleDegrees * fDegreesToRadians) * cosf(fTA));

		// if A = 0, then A = 0.001  else if A > 0, then A = A   else if A < 0, then A = -A
		if( fTiltARadians == 0.0)
			fTiltARadians = 0.001;
		else if( fTiltARadians > 0.0)
			fTiltARadians = fTiltARadians;
		else if( fTiltARadians < 0.0)
			fTiltARadians = -fTiltARadians;

		// B = [Sin (ZN) * Sin (AZ Tilt)]
		fTiltBRadians =  ( sinf(fgPositionInfo.fZenithAngleDegrees * fDegreesToRadians) * sinf(fgPositionInfo.fAzimuthTiltAngleDegrees * fDegreesToRadians));

		 // need to test denominator for zero value BEFORE division
		if(fTiltARadians IS 0.0)			// check for divide by 0
		{

			fgPositionInfo.fModuleTiltAngleDegrees = -90.0;
		}
		else
		{
			// MT = - 1 * ATan (B/A)
			fTiltModule = -1.0 * atanf(fTiltBRadians / fTiltARadians);

			// convert to degrees
			fgPositionInfo.fModuleTiltAngleDegrees = fTiltModule * fRadiansToDegrees;
		}

		return(fgPositionInfo.fModuleTiltAngleDegrees);
	}


	float CalculateBackTrack(PTR_RTCC_DATE_TIME ptrDateTime)
	{
		float fBacktrack_Gain = 0.0;			// adjusted so that panel is flat when sun is at 90 degrees (?)

		// initialize position info fixed values from RAM copy of Flash System Parameters
		fgPositionInfo.fPanelShadowStartAngleDegrees = ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees;
		fgPositionInfo.fSunShadowStartAngleDegrees = ptrRAM_SystemParameters->fSunShadowStartAngleDegrees;

		//fBacktrack_Gain = 45/ (90 â€“ 62)
		if (fgPositionInfo.fSunShadowStartAngleDegrees IS_NOT 90.0)		// this is a check for divide by 0
		{
			fBacktrack_Gain = (fgPositionInfo.fPanelShadowStartAngleDegrees / (90.0 - fgPositionInfo.fSunShadowStartAngleDegrees));    // returns float value only when numerator is float (45.0)
		}

		//fBackTrack(PM) = fSunShadowPanelAngleDegrees - (ABS(Current tilt) - fSunShadowStartHeight) * fBacktrack_Gain
		if( fabs(fgPositionInfo.fModuleTiltAngleDegrees) >= fgPositionInfo.fSunShadowStartAngleDegrees )
		{
			// enable backtracking, corrected per SmartTrak 1 Apr 14
			//fgPositionInfo.fBackTrack = (fgPositionInfo.fPanelShadowStartAngleDegrees - (fabs(fgPositionInfo.fModuleTiltAngleDegrees) - fgPositionInfo.fSunShadowStartAngleDegrees) ) * fBacktrack_Gain;
			fgPositionInfo.fBackTrack = (fgPositionInfo.fSunShadowStartAngleDegrees - (fabs(fgPositionInfo.fModuleTiltAngleDegrees) - ptrRAM_SystemParameters->fSunShadowStartHeight) ) * fBacktrack_Gain;

			// adjust for AM/PM
			if (ptrDateTime->cHours < 12)
			{
				// Backtrack AM is Backtrack PM * -1
				fgPositionInfo.fBackTrack *= -1.0;
			}

		}
		else
		{
			// disable backtracking
			fgPositionInfo.fBackTrack = 0.0;
		}
		// else calculate as ( currnt angle - previsous angle)* fBacktrack_Gain

		return(fgPositionInfo.fBackTrack);
	}
#endif	// USE_SINGLE_POLAR_AXIS


// *************************************************************************************************
//					Complete Sun Position Calculations
// *************************************************************************************************

BOOL CalculateSunPosition(SmartTrakOrientation *ptrOrientation, PTR_RTCC_DATE_TIME ptrDateTime)
{
	// calculation process is broken down into steps for debugging and analysis
	IGNORE_RETURN_VALUE GetDayOfYear(ptrDateTime);
	GetYearDegrees(ptrDateTime);
	IGNORE_RETURN_VALUE CalculateDeclinationAngle();
	IGNORE_RETURN_VALUE CalculateTimeCorrection(ptrDateTime);
	IGNORE_RETURN_VALUE CalculateHourAngle(ptrDateTime);

	IGNORE_RETURN_VALUE CalculateExoZenithAngle();
	IGNORE_RETURN_VALUE CalculateSolarZenithAngle();

	IGNORE_RETURN_VALUE	CalculateExoAzimuthAngle();
	IGNORE_RETURN_VALUE	CalculateSolarAzimuthAngle();

        IGNORE_RETURN_VALUE   CalculateSunElevationAngle();
	IGNORE_RETURN_VALUE   CalculateAzimuthTilt();
	IGNORE_RETURN_VALUE   CalculateModuleTilt();

	#ifdef USE_BACKTRACKING
		// backtracking calculations
		if (ptrRAM_SystemParameters->bBacktrackingEnabled IS TRUE)
		{

			IGNORE_RETURN_VALUE   CalculateBackTrack(ptrDateTime);
		}
	#endif	// USE_BACKTRACKING


	// set SPA orientation
	ptrOrientation->fAzimuth = fgPositionInfo.fAzimuthAngleDegrees;
	ptrOrientation->fElevation = (float)90.0 - fgPositionInfo.fZenithAngleDegrees;

	#ifdef USE_BACKTRACKING
		if (ptrRAM_SystemParameters->bBacktrackingEnabled IS TRUE)
		{
			ptrOrientation->bBacktrackingActive = TRUE;
			ptrOrientation->fSunElevationAngleDegrees = fgPositionInfo.fSunElevationAngleDegrees;
			ptrOrientation->fAzimuthTiltAngleDegrees = fgPositionInfo.fAzimuthTiltAngleDegrees;
			ptrOrientation->fModuleTiltAngleDegrees = fgPositionInfo.fModuleTiltAngleDegrees;
			ptrOrientation->fSunShadowStartAngleDegrees = fgPositionInfo.fSunShadowStartAngleDegrees;	// presently just a fixed value
			ptrOrientation->fBackTrack = fgPositionInfo.fBackTrack;
		}
		else
		{
                        ptrOrientation->bBacktrackingActive = FALSE;
			ptrOrientation->fSunElevationAngleDegrees = fgPositionInfo.fSunElevationAngleDegrees;
			ptrOrientation->fAzimuthTiltAngleDegrees = fgPositionInfo.fAzimuthTiltAngleDegrees;
			ptrOrientation->fModuleTiltAngleDegrees = fgPositionInfo.fModuleTiltAngleDegrees;
			ptrOrientation->fBackTrack = 0.0 ;
		}
	#endif	// USE_BACKTRACKING

	return TRUE;

}



#if defined(USE_BACKTRACKING)

        #define AHEAD_DEGREES                   2
	// mechanical limits, converted from local coordinates to SPA coordinates
        #define	REVERSE_SINGLEAXIS_MAX          -45
        #define	FORWARD_SINGLEAXIS_MAX          45
        #define ADJUST_BACKTRAK(x)              (FORWARD_SINGLEAXIS_MAX - fabs(x))

	#define	REVERSE_MECHANICAL_LIMIT	((ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse - ptrRAM_SystemParameters->fAZ_Offset))
	#define	FORWARD_MECHANICAL_LIMIT	((ptrRAM_SystemParameters->fSingle_SoftLimit_Forward - ptrRAM_SystemParameters->fAZ_Offset))

	BOOL AdjustForBacktracking(SmartTrakOrientation *ptrOrientation)
	{
		if ((ptrOrientation->bBacktrackingActive IS TRUE) AND (ptrOrientation->fBackTrack IS_NOT 0.0))
		{
                        if(ptrOrientation->fAzimuth <= REVERSE_SINGLEAXIS_MAX )
			{
				ptrOrientation->fAzimuth = -ADJUST_BACKTRAK(ptrOrientation->fBackTrack);
                                ptrOrientation->fAzimuth = ptrOrientation->fAzimuth - AHEAD_DEGREES;
                                if(ptrOrientation->fAzimuth <= REVERSE_MECHANICAL_LIMIT)
                                    ptrOrientation->fAzimuth = REVERSE_MECHANICAL_LIMIT;
                                return TRUE;
			}
			else if(ptrOrientation->fAzimuth >= FORWARD_SINGLEAXIS_MAX)
			{
				ptrOrientation->fAzimuth = ADJUST_BACKTRAK(ptrOrientation->fBackTrack);
                                ptrOrientation->fAzimuth = ptrOrientation->fAzimuth - AHEAD_DEGREES;
                                if(ptrOrientation->fAzimuth >= FORWARD_MECHANICAL_LIMIT)
                                    ptrOrientation->fAzimuth = FORWARD_MECHANICAL_LIMIT;
                                return TRUE;

			}
                }				// backtracking is ACTIVE
                else
                {
                         ptrOrientation->fAzimuth = ptrOrientation->fAzimuth + AHEAD_DEGREES;

                          if(ptrOrientation->fAzimuth <= REVERSE_MECHANICAL_LIMIT)
                                    ptrOrientation->fAzimuth = REVERSE_MECHANICAL_LIMIT;

                         else if(ptrOrientation->fAzimuth >= FORWARD_MECHANICAL_LIMIT)
                                    ptrOrientation->fAzimuth = FORWARD_MECHANICAL_LIMIT;

                         return FALSE;
			
                }
	}
#endif

// end of SunPosition.c
