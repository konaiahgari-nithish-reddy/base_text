// *************************************************************************************************
//										S u n P o s i t i o n . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Sun Position Algorithm function declarations
//
// *************************************************************************************************

BOOL CalculateSunPosition(SmartTrakOrientation *ptrOrientation, PTR_RTCC_DATE_TIME ptrDateTime);
#ifdef USE_SINGLE_POLAR_AXIS
	BOOL AdjustForBacktracking(SmartTrakOrientation *ptrOrientation);
#endif


// end of SunPosition.h

