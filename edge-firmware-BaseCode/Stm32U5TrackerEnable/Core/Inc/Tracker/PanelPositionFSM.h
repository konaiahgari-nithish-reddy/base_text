// *************************************************************************************************
//								P a n e l P o s i t i o n F S M . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Sun Position FSM, started by motion Commands
// *************************************************************************************************


// <sek> this needs separate AZIMUTH and ELEVATION HOLDs
#include"Smarttrak.h"
//#include"inclinometer.h"

enum tagPanelPositionMovement
{
	PANEL_POSITION_HOLD,
	PANEL_POSITION_MOVE
};


enum tagPanelPositionMovement PanelPositionFSM(SmartTrakOrientation *ptrOrientation);

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined(DEFINE_GLOBALS)
        GLOBAL ARRAY BYTE	SPS;
#elif defined (DEFINE_EXTERNS)

        GLOBAL ARRAY BYTE	SPS;
#endif
// end of SunPositionFSM.h


