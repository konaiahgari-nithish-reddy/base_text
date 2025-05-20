// *************************************************************************************************
//							g s f s t d . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	general type definitions, convienience macros
//		copyright (c) 2013 gsf Engineering
//
// *************************************************************************************************

// Common header file of project wide #defines

// This is derived from a standard gsf Engineering header file, and is NOT really project specific


/******************************************************************************
*								VARIABLE TYPE DEFINITIONS
******************************************************************************/

// NOTE that the Microchip dsPIC C30 compiler considers a short and int to be 2 bytes, and a long is 4 bytes
//typedef unsigned char	    BYTE;		    // Unsiged, 8 bit (Alternative to below)
//typedef unsigned char	    BOOL;		    // A boolean T/F value (C30 Does not handle booleans)
//typedef unsigned int	    WORD;		    // Unsigned, 16 bit
typedef unsigned char       INT8U;          // Unsigned, 8 bit
typedef unsigned short		INT16U;         // Unsigned, 16 bit
typedef unsigned long	    INT32U;		    // Unsigned, 32 bit
typedef signed char		    INT8S;		    // Signed eight bit
typedef signed short		INT16S;		    // Signed 16 bit
typedef signed long		    INT32S;		    // Signed 32 bit
typedef void (*FUNC_PTR)(void);			    // Function Pointer
typedef unsigned int*	    REG_PTR;		// Register address pointer.

typedef WORD                EVENTFLAGS;     // Eventflag register
#define	NO_EVENTFLAGS		(EVENTFLAGS)0	// for unsigned comparison

typedef	INT16S				ENUM;			// enums are by definition ints, although C30 allows smaller sizes...


/******************************************************************************
*								DEFINITIONS
******************************************************************************/
#ifndef NULL
	#define	NULL		    0x0000
#endif /* NULL */

#define FALSE			(BOOL)0					// C30 Does not handle booleans
#define TRUE			(BOOL)1					// C30 Does not handle booleans
#define SUCCESS		    0
#define FAILURE		    -1
#define ERROR			-1
//#define BUSY			1
#define LOW				0
#define HIGH			1
//#define DONE			1
//#define NOT_DONE		0

// substate completion flags, used in FSMs
//#define	SUBSTATE_NOT_DONE	FALSE
//#define	SUBSTATE_DONE		TRUE
#define SUBSTATE_NOT_DONE		0
#define SUBSTATE_DONE			1
#define	SUBSTATE_DONE_FAIL		2
#define SUBSTATE_DONE_TIMEOUT	3

#define PASS			((BOOL) 1)
#define FAIL			((BOOL) 2)
#define INFINITE_LOOP	TRUE


//#define FOREVER			0xffff
#define bNO				(BOOL)0
#define bYES			(BOOL)1

#define bSTOP			0
#define bGO				1
#define bSTART			1

#define bOFF			0
#define bON				1

#define bBAD			0
#define bGOOD			1

#define bRESET			0
#define bSET			1
#define bTEST			2

// specific character definitions
#define	ASCII_CR			0x0D		// CR conflicts with 8051 internal Bit name!
#define	ASCII_LF			0x23
#define	ASCII_ESC			0x1B
#define	ASCII_PERIOD		'.'
#define	ASCII_MINUS			'-'
#define ASCII_SPACE			' '

/******************************************************************************
*								   MACROS
******************************************************************************/
// bit handling macros - limited to 16 bit numbers (uints)

#define BITSET(var, eBitPos)				((var) |= 1<<((int)eBitPos))
#define BITCLEAR(var, eBitPos)				((var) &= ~(1<<((int)eBitPos)))
#define BITMASK(eBitPos)			        (1<<((int)eBitPos))
#define BITTOGGLE(var, eBitPos)				((var) ^= 1<<((int)eBitPos))
#define	IS_BITSET(var, eBitPos)				((var & (1<<((int)eBitPos))) ? TRUE : FALSE)
#define	IS_BITCLEAR(var, eBitPos)			((var & (1<<((int)eBitPos))) ? FALSE : TRUE)

#define LBITMASK(eBitPos)			        (INT32U)(1L<<((int)eBitPos))

#define TW0_BYTES_TO_WORD(high, low)        (((high << 8) & 0xFF00) | low)
#define EXTRACT_HIGH_FROM_WORD(word)        ((word & 0xFF00) >> 8)
#define EXTRACT_LOW_FROM_WORD(word)         (word & 0x00FF)

#define	ABS(x)								( ((x) > 0) ? (x) : -(x) )
#define	FABS(x)								( ((x) > 0.0) ? (x) : -(x) )


/*lint -e723 */
#define     IS              ==
/*lint +e723 */

#define     IS_NOT          !=
#define     AND             &&
#define     OR              ||
#define     NOT             !
#define     TOGGLE          !

#define     IS_TRUE                         // logical test for TRUE
//lint -e723 error 723: (Info -- Suspicious use of =)
#define     IS_FALSE		== 0			// logical test for FALSE
//lint +e723

#define		ZERO			0
#define     IS_ZERO			IS ZERO
#define     IS_NOT_ZERO		// compare to non-zero

#define     SZ_TERM         (const char) '\0'   // string terminator


/*lint -e760 (Info -- Redundant macro 'PRIVATE' defined identically at same file location) */
#ifndef DEBUG_STATICS			// simple macro to allow static functions and data to be made visible to debugger
	#define	PRIVATE		    	static
	#define	PRIVATE_INIT	    static
#else
	#define	PRIVATE
	#define	PRIVATE_INIT
#endif
/*line +e770 */

/*lint -e767 error 767: (Info -- macro was defined differently in another module */
#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif
#if defined(DEFINE_GLOBALS)		// simple macro to allow global variables to be placed in a common header file
	#define	GLOBAL
	#define	GLOBAL_INIT
#elif defined (DEFINE_EXTERNS)
	#define	GLOBAL					extern
	//#define	GLOBAL_INIT
#endif
/*lint +e767 */

#define	LOCAL						auto
#define	LOCAL_INIT					auto

#define	PERSISTENT_LOCAL			static		// auto is implied
#define	PERSISTENT_LOCAL_INIT		static		// auto is implied, MUST be initialized


#define	ARRAY
#define	FILE_GLOBAL
#define	FILE_GLOBAL_INIT


#define	IGNORE_RETURN_VALUE		(void)			// flag functions that have a return value, but it is not checked
#define	VARIABLE_NOT_REFERENCED	(void)			// keep compiler quiet with variables that are not referenced

// comment macros to identify potentially problematic code
#define	BLOCKING_WAIT_FOR_HARDWARE				// flag untimed loops that wait for hardware register value to change
#define	BLOCKING_DELAY							// flag loops that hog the process as a simplistic delay

// end of gsfstd.h
