// *************************************************************************************************
//										S e r i a l D i s p l a y . C
// *************************************************************************************************
//
//		Project:	String Monitoring
//
//		Contains:	Serial Display Code for Debug and Application
//
// *************************************************************************************************

//-----------------------------------------------------------------------------
// #include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

// processor include file
//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include <string.h>				// Microchip string functions
//#include <ctype.h>				// tolower()

// gsf standard #defines, use <> so lint will treat this as a library file
#include "gsfstd.h"				// gsf standard #defines

#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "SerialPort.h"
#include "StrConversions.h"			// ASCII string <==> numeric conversions
//#include "Timer.h"				// Timer access functions
//#include "Debounce.h"			// Input Switch debounce functions
#include "SerialDisplay.h"

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagSerialDisplayErrors
{
	SER_DISPLAY_ERROR_NONE = SER_DISPLAY_ERROR_BASE,
	SER_DISPLAY_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	SER_DISPLAY_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	SER_DISPLAY_ERROR_INVALID_STATE,			// 3 not a valid state
	SER_DISPLAY_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	SER_DISPLAY_ERROR_BUFFER_OVERFLOW			// 5 menu string too long
};


//-------------------------------------------------------------------------------------------------------
// File Global data
//-------------------------------------------------------------------------------------------------------

const char * strNewLine = "!";

FILE_GLOBAL ARRAY char	fgszDisplayStr[DISPLAY_STR_SIZE + 1];		// global display string buffer

//-------------------------------------------------------------------------------------------------------
// Display string handlers
//-------------------------------------------------------------------------------------------------------

// *************************************
//		RAM string handlers
// *************************************

void ClearDisplayStr(void)
	{
	fgszDisplayStr[0] = '\0';
	}

void AddDisplayStr(const char *pStr)
	{
	strcat(fgszDisplayStr, pStr);

	// make sure we have not overrun the menu string buffer
	if (strlen(fgszDisplayStr) > DISPLAY_STR_SIZE)
		{
		fgszDisplayStr[DISPLAY_STR_SIZE] = '\0';
		RuntimeError(SER_DISPLAY_ERROR_BUFFER_OVERFLOW);
		}
	}

void AddDisplayStrAndNewLine(const char *pStr)
	{
	strcat(fgszDisplayStr, pStr);

	// add a line terminator from a ROM string
	IGNORE_RETURN_VALUE strcat(fgszDisplayStr, "\r\n");

	// make sure we have not overrun the menu string buffer
	if (strlen(fgszDisplayStr) > DISPLAY_STR_SIZE)
		{
		fgszDisplayStr[DISPLAY_STR_SIZE] = '\0';
		RuntimeError(SER_DISPLAY_ERROR_BUFFER_OVERFLOW);
		}
	}


void AddDisplayTab(void)
	{
	AddDisplayStr("\t");
	}

void AddDisplaySpace(void)
	{
	AddDisplayStr(" ");
	}

void AddDisplayNewLine(void)
	{
	AddDisplayStr(strNewLine);
	}


void DisplayStr(UART_MODULE UARTid)
	{
	StartTransmitString(UARTid, fgszDisplayStr);
	}

void DisplayStrWait(UART_MODULE UARTid)
	{
	StartTransmitString(UARTid, fgszDisplayStr);

	// this is a BLOCKING display sequence

	// wait for serial TX to finish
	while(IsDisplayDone(UARTid) IS bNO)
		BLOCKING_WAIT_FOR_HARDWARE;

	}

// *************************************
//	Placekeeper	string handler
// *************************************

// Displays a string with a changing placekeeper character at the end

// this is setup as an array of pointers
FILE_GLOBAL ARRAY const char *pstrSequenceText[] =
	{
	"|",
	"/",
	"-",
	"\\",
	"*",
	""
	};

#define MAX_SEQ_INDEX		4
BYTE nSeqIndex = 0;

void DisplayStrSequence(UART_MODULE UARTid, char *pStr)
	{
	// copy from ROM string to RAM buffer
	IGNORE_RETURN_VALUE strcpy(fgszDisplayStr, pStr);							// copy ROM text to output buffer
	IGNORE_RETURN_VALUE strcat(fgszDisplayStr, pstrSequenceText[nSeqIndex]);	// add sequence character
	++nSeqIndex;						// bump sequence index
	if (nSeqIndex > MAX_SEQ_INDEX)		// bounds check
		nSeqIndex = 0;					// restart sequence

	IGNORE_RETURN_VALUE strcat(fgszDisplayStr, "\r");							// add line terminator

	// make sure we have not overrun the menu string buffer
	if (strlen(fgszDisplayStr) > DISPLAY_STR_SIZE)
		{
		fgszDisplayStr[DISPLAY_STR_SIZE] = '\0';
		RuntimeError(SER_DISPLAY_ERROR_BUFFER_OVERFLOW);
		}

	StartTransmitString(UARTid, fgszDisplayStr);
	}


// *************************************************************************************************

// *************************************************************************************************

// *********************************************************
//			General Message Display
// *********************************************************

// currently used in MenuFSM() ONLY for error messages
void stricpy(char *pStr1,char *pStr2,UINT8 len)
{
    while(len  > 0)
    {
        *pStr1++ = *pStr2++;
        len--;
    }
    *pStr1 = '\0';
    *pStr1 = '\0';
}
void SendMBMessage(UART_MODULE UARTid, const char *pStr, UINT8 len,enum tagWaitForDisplay eWaitForDisplay)
{
    stricpy(fgszDisplayStr, pStr, len);
    TXLen[UARTid] = 0;
    StartTransmitString(UARTid, fgszDisplayStr);


    if (eWaitForDisplay IS WAIT_FOR_DISPLAY)
		{
		// this is a BLOCKING display sequence

		// wait for serial TX to finish
		while(IsDisplayDone(UARTid) IS bNO)
			BLOCKING_WAIT_FOR_HARDWARE;

		// clear the display string buffer for subsequent usage
		ClearDisplayStr();
		}
}
void DisplayMessage(UART_MODULE UARTid, const char *pStr, enum tagWaitForDisplay eWaitForDisplay)
	{
	if (eSerialOutputMode IS_NOT SER_MODE_MENU)
		return;

	ClearDisplayStr();
	AddDisplayStr(pStr);
	AddDisplayNewLine();
	DisplayStr(UARTid);

	if (eWaitForDisplay IS WAIT_FOR_DISPLAY)
		{
		// this is a BLOCKING display sequence

		// wait for serial TX to finish
		while(IsDisplayDone(UARTid) IS bNO)
			BLOCKING_WAIT_FOR_HARDWARE;

		// clear the display string buffer for subsequent usage
		ClearDisplayStr();
		}
	}

// *********************************************************
//			Single Character Display (Echo)
// *********************************************************

// this is used for echoing parameter entry characters

void DisplayCharacter(UART_MODULE UARTid, const char cChar, enum tagWaitForDisplay eWaitForDisplay)
	{
	LOCAL ARRAY char strEchoChar[2];	// string for echoing parameter characters, one at a time

	if (eSerialOutputMode IS_NOT SER_MODE_MENU)			// only used for Serial Menu
		return;

	ClearDisplayStr();							// initialize output buffer

	strEchoChar[0] = cChar;						// copy single character to output buffer
	strEchoChar[1] = SZ_TERM;					// terminate the buffer; all serial output is based on terminated strings

	AddDisplayStr(strEchoChar);					// copy string (just one character!) to output buffer
	DisplayStr(UARTid);

	if (eWaitForDisplay IS WAIT_FOR_DISPLAY)
		{
		// this is a BLOCKING display sequence

		// wait for serial TX to finish
		while( IsDisplayDone(UARTid) IS bNO)
			BLOCKING_WAIT_FOR_HARDWARE;

		// clear the display string buffer for subsequent usage
		ClearDisplayStr();
		}
	}


// *********************************************************
//			Check for Display Complete
// *********************************************************

BOOL IsDisplayDone(UART_MODULE UARTid)
{
	if (eSerialOutputMode IS_NOT SER_MODE_MENU)			// only used for Serial Menu
		return TRUE;

	// wait for serial TX to finish
	if (IsSerialTxComplete(UARTid) IS bNO)
		return FALSE;

	// display must be complete
	// clear the display string buffer for subsequent usage
	ClearDisplayStr();
	return TRUE;
}



// *************************************************************************************************
//								RealTime Message Display Functions
// *************************************************************************************************

// *********************************************************
//			RealTime Message Display
// *********************************************************

// this function is used only for real-time display of packets and error messages

void RealTimeDisplayMessage(UART_MODULE UARTid, const char *pStr)
{

	// if RealTime display is not enabled, just return without doing anything
//	if (bRealTimeDisplay IS FALSE)
	if (eSerialOutputMode IS_NOT SER_MODE_REALTIME)
		return;


	// RealTime display is enabled, so display the message and do NOT wait for it to complete
	DisplayMessage(UARTid, pStr, NO_WAIT_FOR_DISPLAY);

}


// *********************************************************
//			RealTime Constructed String Display
// *********************************************************

// this function is used only for real-time display of status and error messages
// NOTE: this function makes use of the global fgszDisplayStr,
// so the caller must be sure NOT to modify the string until the display is complete

void RealTimeDisplayStr(UART_MODULE UARTid)
{

	// if RealTime display is not enabled, just return without doing anything
//	if (bRealTimeDisplay IS FALSE)
	if (eSerialOutputMode IS_NOT SER_MODE_REALTIME)
		return;


	// RealTime display is enabled, so display the contents of fgszDisplayStr and do NOT wait for it to complete
	DisplayStr(UARTid);

}


// *********************************************************
//			Check for Display Complete
// *********************************************************

BOOL IsRealTimeDisplayDone(UART_MODULE UARTid)
{

	// if RealTime display is not enabled, just return without doing anything
	// Fake TRUE return because display will not even occur
//	if (bRealTimeDisplay IS FALSE)
	if (eSerialOutputMode IS_NOT SER_MODE_REALTIME)
		return TRUE;

	// RealTime display is enabled

	// wait for serial TX to finish
	if (IsSerialTxComplete(UARTid) IS bNO)
		return FALSE;

	// display must be complete
	// clear the display string buffer for subsequent usage
	ClearDisplayStr();

	return TRUE;
}


// end of DisplayString.c
