// *************************************************************************************************
//										S e r i a l D i s p l a y . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Serial Display Code for Debug and Application definitions and declarations
//
// *************************************************************************************************

//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

enum tagWaitForDisplay
	{
		NO_WAIT_FOR_DISPLAY = 0,
		WAIT_FOR_DISPLAY
	};


// functions to format fgszDisplayStr[]
void ClearDisplayStr(void);
void AddDisplayStr(const char *pStr);
void AddDisplayStrAndNewLine(const char *pStr);
void AddDisplayNewLine(void);
void AddDisplayTab(void);
void AddDisplaySpace(void);

// functions to display fgszDisplayStr[] (send to serial port)
void DisplayStr(UART_MODULE UARTid);
void DisplayStrWait(UART_MODULE UARTid);

// functions to display a caller supplied string
void DisplayCharacter(UART_MODULE UARTid, const char cChar, enum tagWaitForDisplay eWaitForDisplay);
void DisplayMessage(UART_MODULE UARTid, const char *pStr, enum tagWaitForDisplay eWaitForDisplay);

// display a 'waiting' sequence of changing characters
void DisplayStrSequence(UART_MODULE UARTid, char *pStr);

// real-time data display
void RealTimeDisplayMessage(UART_MODULE UARTid, const char *pStr);
void RealTimeDisplayStr(UART_MODULE UARTid);

// check for end of string AND hardware done
BOOL IsDisplayDone(UART_MODULE UARTid);
BOOL IsRealTimeDisplayDone(UART_MODULE UARTid);


#define	DISPLAY_LINE_SIZE		200									// maximum displayable line in Hyperterminal
#define	DISPLAY_STR_SIZE		(DISPLAY_LINE_SIZE * 3)				// long enough for just THREE lines and a terminator - which should ALWAYS be long enough! (Move menu is the longest)

#ifdef USE_REMOTE_COMMANDS
	#define	INPUT_BUFFER_SIZE	40								// large enough for all remote command strings
#else
	#define	INPUT_BUFFER_SIZE	11								// larger than needed for any reasonable numeric input
#endif
#define	INPUT_STR_SIZE			(size_t)(INPUT_BUFFER_SIZE - 1)		// long enough for just ONE line and a terminator - which should ALWAYS be long enough!


//-------------------------------------------------------------------------------------------------------
// Global Data
//-------------------------------------------------------------------------------------------------------

#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined(DEFINE_GLOBALS)
	GLOBAL_INIT	BOOL	bRealTimeDisplay = FALSE;				// flag to enable/disable real time message display
	GLOBAL_INIT	enum	tagSerialOutputMode eSerialOutputMode = SER_MODE_UNINITIALIZED;

	GLOBAL ARRAY char	pgcInputBuffer[INPUT_BUFFER_SIZE];		// keystroke input buffer, much bigger than we need
        GLOBAL ARRAY char	pgcMBBuffer[INPUT_BUFFER_SIZE];
#elif defined (DEFINE_EXTERNS)
	GLOBAL	BOOL		bRealTimeDisplay;
	GLOBAL	enum		tagSerialOutputMode eSerialOutputMode;
	GLOBAL ARRAY char	pgcInputBuffer[];
        GLOBAL ARRAY char	pgcMBBuffer[];
#endif

// end of SerialDisplay.h




