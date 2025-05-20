// *************************************************************************************************
//										R T C C . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	RTCC Function declarations
//
// *************************************************************************************************
#ifndef RTCC_H
	#define RTCC_H
#endif

#ifndef DS3232_H
	#error DS3232.h must be #included first
#endif

BOOL FormatRTCCDateTime(char *ptrOutputStr, PTR_RTCC_DATE_TIME ptrDateTime);

// end of RTCC.h
