// *************************************************************************************************
//										M e n u F S M . H
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Debug Menu function definitions
//
// *************************************************************************************************

void MenuFSM(UART_MODULE UARTid);
void CurrentMechanicalOrientation_Format(char *ptrOutputStr);
void CurrentLocalOrientation_Format(char *ptrOutputStr);							// accounts for offsets
void Orientation_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);	  // general purpose
void SPA_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);				// general purpose
void SPA_Backtrack_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#ifdef DEFINE_GLOBALS
	GLOBAL_INIT BOOL	pgbTerminate = FALSE;
	GLOBAL_INIT float	fgfCLMoveDistance = 0.0;
	GLOBAL_INIT float	fgfMSStepSize = 0.0;
	GLOBAL_INIT float	fgfMoveToXCoord = 0.0;
	GLOBAL_INIT float	fgfMoveToYCoord = 0.0;
#elif defined (DEFINE_EXTERNS)
	GLOBAL BOOL pgbTerminate;
	GLOBAL float	fgfCLMoveDistance;
	GLOBAL float	fgfMSStepSize;
	GLOBAL float	fgfMoveToXCoord;
	GLOBAL float	fgfMoveToYCoord;
#endif


// end of MenuFSM.h
