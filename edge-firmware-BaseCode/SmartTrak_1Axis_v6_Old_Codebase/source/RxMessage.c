// *************************************************************************************************
//								R x M e s s a g e . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Remote Message/Command handler functions
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

#include "HardwareProfile.h"

#include <stdlib.h>
#include <string.h>				// for memcpy()
#include <stdio.h>
#include <ctype.h>
#include <limits.h>
#include <stdbool.h>

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions, default Parameter table

#include "EventFlags.h"			// event flag definitions and globals
#include "SerialPort.h"
#include "SerialDisplay.h"		// display functions for menus
#include "MenuFSM.h"

#include "RxMessage.h"
#include "UpdateParameters.h"

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MoveSequenceFSM.h"	// move sequence FSM


#include "SST25VF016.h"			// SPI Flash function definitions
#include "DS3232.h"				// DS3232 RTCC function definitions
#include "RTCC.h"				// DS3232 RTCC RAM function definitions
#ifdef USE_PCA9554_IO			// enable in config.h ONLY if PCA9554 hardware is present
	#include "PCA9554.h"		// SPI expander definitions
#endif	// USE_PCA9554_IO
#ifdef USE_MMA8452Q_INCLINOMETER
	#include "mma845x.h"         // MMA845xQ macros
	#include "Inclinometer.h"
#endif	//  USE_MMA8452Q_INCLINOMETER
#include "ADC10Read.h"
#include "BMS.h"

#include "Debounce.h"			// Input Switch debounce functions
#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"

#include "StrConversions.h"
#include "CoordTranslate.h"		// coordinate translation and formatting
#include "Zigbee.h"
// ***********************************************
//				Definitions
// ***********************************************

enum tagUpdateParameterErrors
{
	RX_MSG_ERROR_NONE = RX_MSG_ERROR_BASE,
	RX_MSG_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	RX_MSG_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	RX_MSG_ERROR_INVALID_STATE,				// 3 not a valid state
	RX_MSG_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	RX_MSG_ERROR_INVALID_SELECTION,			// 5 not a valid selection
	RX_MSG_ERROR_UNKNOWN_COMMAND,			// 6 not a valid command
	RX_MSG_ERROR_INVALID_PARAMETER,			// 7 not a valid parameter selection
	RX_MSG_ERROR_OUT_OF_RANGE_PARAMETER,	// 8 not a valid parameter selection
	RX_MSG_ERROR_BUFFER_OVERFLOW,			// A menu string too long
	RX_MSG_ERROR_RTCC_READ_ERROR,			// B RTCC read failure
	RX_MSG_ERROR_INCL_READ_ERROR,			// C Inclinometer read failure

	RX_MSG_UNPROCESSED_EVENT = RX_MSG_ERROR_BASE + 0x0F
};

//-----------------------------------------------
// Rx Messagel FSM states
enum tagRxMsgStates
{
    ST_RX_MSG_INIT,							// initial state, only at power up
	ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR,	// collecting keystrokes and waiting for CR terminator
	ST_RX_MSG_PROCESS_MSG,
	ST_RX_MSG_WAIT_FOR_CMD,
	ST_RX_MSG_RESPONSE

	//ST_RX_MSG_DISPLAY_REALTIME_DATA		// realtime streaming data is generated elsewhere, and is only controlled (on/off) by MenuFSM

};

enum tagCommand
{
	COMMAND_NONE,
	// Commands
	COMMAND_RUN_UP,
	COMMAND_RUN_EAST,
	COMMAND_RUN_WEST,
	COMMAND_RUN_DOWN,
	COMMAND_STOP,
        COMMAND_START,
	COMMAND_CALIBRATE,
	COMMAND_FORCE_CURRENT_ORIENTATION,

	COMMAND_GET_CURRENT_ORIENTATION,
	COMMAND_GET_SPA_VALUES,
	COMMAND_CLEAR_CURRENT_ORIENTATION,
	COMMAND_SET_TRACKING_MODE,
	COMMAND_SET_SERVICE_MODE,
	COMMAND_SET_MENU_MODE,
	COMMAND_RESET,
        COMMAND_ST_DAYS,
        COMMAND_RCV_DAYS,
        COMMAND_SET_REMOTE_MODE,
        COMMAND_SET_MANUAL_MODE,
        COMMAND_SET_STOW_MODE,
        COMMAND_SET_WIND_STOW,
        COMMAND_SET_WIND_DISS,
        COMMAND_SEND_XBEE,
        COMMAND_SEND_LIMITS,
        COMMAND_RCV_ENV,
        COMMAND_RCV_Lat,
        COMMAND_RCV_Lon,
        COMMAND_RCV_Alt,
        COMMAND_RCV_TZ,
        COMMAND_RCV_STS,
        COMMAND_RCV_SLF,
        COMMAND_RCV_SLR,
        COMMAND_BT_ENB,
        COMMAND_BT_PANEL,
        COMMAND_BT_SUN,
        COMMAND_SEND_ATTR,
        COMMAND_RECV_RTCC,
        COMMAND_RECV_Hour,
        COMMAND_RECV_Min,
        COMMAND_RECV_Sec,
        COMMAND_RECV_Day,
        COMMAND_RECV_Date,
        COMMAND_RECV_Month,
        COMMAND_RECV_Year,

};

// ***********************************************
//		File Global Data
// ***********************************************

UINT8 fgcParameterFlags = 0x00;						// collects (bit-wise OR) Attribute flags while processing messages

PRIVATE_INIT enum tagRxMsgStates stRxMsgState = ST_RX_MSG_INIT;
PRIVATE_INIT enum tagRxMsgStates stRxMsgServiceState = ST_RX_MSG_INIT;
PRIVATE_INIT BYTE bRxMsgFSMSequenceCtr = 0;					// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus = SUBSTATE_NOT_DONE;		// substate status flag
PRIVATE_INIT BYTE recvflag = 0;
PRIVATE_INIT BYTE chkflag = 0;
FILE_GLOBAL ARRAY char szReturnStr[DISPLAY_LINE_SIZE + 1];

char strnicmp(const char* pStr1, const char *pStr2, size_t Count);

// =======================================
//            Globals
// =======================================

//***********************************************
//			Forward References
//***********************************************

BOOL RxMessageHandler(char *ptrBuffer);
BOOL ParameterHandler(char *ptrBuffer, UINT8 bTblIndex);
BOOL CommandHandler(char *ptrBuffer, UINT8 bTblIndex);


// =======================================
//				Externs
// =======================================


//*************************************************************************************************
//									Remote Command Jump Table
//*************************************************************************************************
// Definition of jump table for decoding and processing command and parameter data messages.

// The jump table below defines ALL available commands and parameters. Each is identified by a distinct message name string.
// The jump table defines what to DO with the message when it is received.
// Requests to set or get (return) a parameter value are handled by a small number of handler functions, depending on the data type.
// Each command identifies a command handler function.

// Messages are either pure ASCII strings
// The general formats of ASCII messages is:
//
//		<string>										may be a parameter-less command OR a request to return parameter value
//		<string> <ASCII parameter>						may be command with parameter, or request to set parameter (only one value allowed for setting a parameter)
//		<string> <ASCII parameter>	<ASCII parameter>	may be command with parameters

// Each successfully processed command is followed by a response
//		OK												does not generate a return value
//		<string>  <ASCII parameter>						generates a return value


// Attribute flag bits
#define	FL_NONE						0x0000
#define	FL_FLASH_STORAGE			0x0001
#define	FL_FLASH_UPDATE				0x0002
#define	FL_SYSTEM_PARAMETER			0x0004
#define	FL_MENU_PARAMETER			0x0008
#define	FL_TRACKING_PARAMETER		0x0010
#define	FL_SYSTEM_FUNCTION			0x0020
#define	FL_TRACKING_FUNCTION		0x0040

#define	FL_PARAM_TYPE_HEX_STRING	0x0100			// flag to indicate parameter is an ASCII HEXl string
#define	FL_PARAM_TYPE_DEC_STRING	0x0200			// flag to indicate parameter is an ASCII Decimal string
#define	FL_PARAM_TYPE_FLOAT_STRING	0x0400			// flag to indicate parameter is an ASCII Float string
#define	FL_PARAM_TYPE_STRING		0x0800			// flag to indicate parameter is an ASCII string
#define	FL_PARAM_TYPE_RTCC_STRING	0x1000			// flag to indicate parameter is an ASCII RTCC value string
#define	FL_PARAM_TYPE_HEX			0x2000			// flag to indicate attribute is hex number rather than a string (not currently supported)

#define	FL_COMMAND					0x8000			// flag to indicate this is a command

#define add_ext
// Definition of jump table for decoding data message commands.
// <sek> NOTE table is stored in CODE space

typedef struct jumpTable
{
    char	*pstrMsgName;						// pointer to Command or attribute name to match. (note that the string itself will not be in the structure, just somewhere....)_
    void	(*ptrFunction)(char *, UINT8);		// Function pointer, either general parameter handler or specific command handler
	enum tagParameter eParameter;
	UINT16	flags;								// flag bits (see above)
} const jumpTable_t;

/** Jump table helper macros
 */
// sizeof(PTR_FLASH_SYSTEM_PARAMETERS-> ## parm) takes a pointer to the instantiated structure and the structure member name
// offsetof(FLASH_SYSTEM_PARAMETERS, parm) takes the STRUCTURE definition and structure member name


// these macros are somewhat redundant, but use of different macros for each data type will allow for flexibility for changes in the future

#define HEX_STR_PARAMETER(strMsgName, eParameter, flags)	{ #strMsgName, (void (*)(char *, UINT8))ParameterHandler, eParameter, ((flags) | FL_PARAM_TYPE_HEX_STRING)}

#define DEC_STR_PARAMETER(strMsgName, eParameter, flags)	{ #strMsgName, (void (*)(char *, UINT8))ParameterHandler, eParameter, ((flags) | FL_PARAM_TYPE_DEC_STRING)}

#define FLOAT_STR_PARAMETER(strMsgName, eParameter, flags)	{ #strMsgName, (void (*)(char *, UINT8))ParameterHandler, eParameter, ((flags) | FL_PARAM_TYPE_FLOAT_STRING)}

#define STR_PARAMETER(strMsgName, eParameter, flags)		{ #strMsgName, (void (*)(char *, UINT8))ParameterHandler, eParameter, ((flags) | FL_PARAM_TYPE_STRING)}

#define RTCC_STR_PARAMETER(strMsgName, eParameter, flags)	{ #strMsgName, (void (*)(char *, UINT8))ParameterHandler, eParameter, ((flags) | FL_PARAM_TYPE_RTCC_STRING)}

// <sek> NOTE the 3rd member is a pointer to a function, the last UINT8 parameter is a dummy for flags
#define CMD_PARAMETER(strMsgName, eCommand, flags)			{ #strMsgName, (void (*)(char *, UINT8))CommandHandler, eCommand, ((flags) | FL_COMMAND)}



/** Jump table for decoding SYSTEM and APPLICATION data message commands.
 *
 * <sek> NOTE:
 * See the above definitions of what the xxx_PARAMETER macros actually generate; not everything appears in the table below
 * the 1st parameter is the name of the Parameter or Command, as used in communication messages. There should be some upper limit on the size of these.
 * the 2rd parameter is a pointer to a function
 * the 3th parameter is a set of flags used in processing the parameter
 * additional parameters are implied by the leading macro
 *
 */

// each structure takes about 20 bytes

ARRAY static const jumpTable_t MsgJumpTable[]=
{

	// ****************************
	//		System
	// ****************************

	// SYSTEM Attributes and Commands
//    CMD_PARAMETER(UID,							UpdateUserID,					FL_SYSTEM_FUNCTION),

//    STR_PARAMETER(PWD,							PARAMETER_PASSWORD,				FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),
//	DEC_STR_PARAMETER(LED_State,				PARAMETER_LED_STATE,			FL_SYSTEM_PARAMETER),

	// ***************************************
	//			Open Loop Menu
	// ***************************************
	DEC_STR_PARAMETER(EL_OpenLoopDutyCycle,		PARAMETER_ELEVATION_DUTY_CYCLE,		FL_MENU_PARAMETER),
	DEC_STR_PARAMETER(AZ_OpenLoopDutyCycle,		PARAMETER_AZIMUTH_DUTY_CYCLE,		FL_MENU_PARAMETER),

	// ***************************************
	//			Closed Loop Menu
	// ***************************************
	FLOAT_STR_PARAMETER(CLMoveDistance,			PARAMETER_MOVE_DISTANCE,		FL_MENU_PARAMETER),

	// ***************************************
	//			Move Sequence Menu
	// ***************************************
	FLOAT_STR_PARAMETER(MSStepSize,				PARAMETER_STEP_SIZE,			FL_MENU_PARAMETER),
	FLOAT_STR_PARAMETER(MoveToXCoord,			PARAMETER_X_COORD,				FL_MENU_PARAMETER),
	FLOAT_STR_PARAMETER(MoveToYCoord,			PARAMETER_Y_COORD,				FL_MENU_PARAMETER),

	// ****************************
	//		unit location
	// ****************************
	//					Message String			Parameter Name enum				Flags
	FLOAT_STR_PARAMETER(Latitude,				PARAMETER_LATITUDE,				FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// R/W, In degrees. Range = +90 degrees to -90 degrees, Positive for north of the equator.
	FLOAT_STR_PARAMETER(Longitude,				PARAMETER_LONGITUDE,			FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// R/W, In degrees. Range = -180 to +180deg, Positive for east of GMT
	FLOAT_STR_PARAMETER(Altitude,				PARAMETER_ALTITUDE,				FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W In meters. Above sea level.
	FLOAT_STR_PARAMETER(Refraction,				PARAMETER_REFRACTION,			FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W No units. Value is around 1.
	FLOAT_STR_PARAMETER(TimeZone,				PARAMETER_TIMEZONE,				FL_SYSTEM_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Offset from GMT (like 5.5 for India)

	DEC_STR_PARAMETER(Tracking_Mode,			PARAMETER_TRACKING_MODE,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// (enum) R/W Values: Manual, Tracking, (Night Stow, Wind Stow)

	// ****************************
	//			Azimuth
	// ****************************
	FLOAT_STR_PARAMETER(AZ_Offset,				PARAMETER_AZ_OFFSET,			FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Azimuth offset from 0.0 at due south Units: degrees
	FLOAT_STR_PARAMETER(AZ_SoftLimit_Reverse,	PARAMETER_AZ_SOFT_LIMIT_REV,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Azimuth soft limit REVERSE (minimum). Units: degrees
	FLOAT_STR_PARAMETER(AZ_SoftLimit_Forward,	PARAMETER_AZ_SOFT_LIMIT_FWD,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Azimuth soft limit FORWARD (maximum). Units: degrees
	FLOAT_STR_PARAMETER(AZ_DeadBand,			PARAMETER_AZ_DEAD_BAND,			FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W TBD
	FLOAT_STR_PARAMETER(AZ_NightStowThreshold,	PARAMETER_AZ_NIGHT_STOW_THRESHOLD,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// Float R/W Elevation threshold to move to/exit NightStow
	FLOAT_STR_PARAMETER(AZ_NightStowPosition,	PARAMETER_AZ_NIGHT_STOW_POS,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Default = 0 (remain in place)
	FLOAT_STR_PARAMETER(AZ_WindStowPosition,	PARAMETER_AZ_WIND_STOW_POS,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Default = 0 (remain in place)

	// ****************************
	//			Elevation
	// ****************************
	FLOAT_STR_PARAMETER(EL_Offset,				PARAMETER_EL_OFFSET,			FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Elevation offset from 45.0 at center Units: degrees
	FLOAT_STR_PARAMETER(EL_SoftLimit_Reverse,	PARAMETER_EL_SOFT_LIMIT_REV,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Elevation soft limit Reverse (minimum). Units: degrees
	FLOAT_STR_PARAMETER(EL_SoftLimit_Forward,	PARAMETER_EL_SOFT_LIMIT_FWD,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Elevation soft limit Forward (maximum). Units: degrees
	FLOAT_STR_PARAMETER(EL_DeadBand,			PARAMETER_EL_DEAD_BAND,				FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// Float R/W TBD
	FLOAT_STR_PARAMETER(EL_NightStowThreshold,	PARAMETER_EL_NIGHT_STOW_THRESHOLD,	FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// Float R/W Elevation threshold to move to/exit NightStow
	FLOAT_STR_PARAMETER(EL_NightStowPosition,	PARAMETER_EL_NIGHT_STOW_POS,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// Float R/W Default = 0 (remain in place)
	FLOAT_STR_PARAMETER(EL_WindStowPosition,	PARAMETER_EL_WIND_STOW_POS,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),		// Float R/W Default = 0 (remain in place)

	// ****************************
	//		Backtracking
	// ****************************
	#ifdef USE_BACKTRACKING
		DEC_STR_PARAMETER(BacktrackingActive,				PARAMETER_BACKTRACKING_ENABLED,			FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// Bool R/W		True: backtracking is enabled
		FLOAT_STR_PARAMETER(PanelShadowStartAngleDegrees,	PARAMETER_PANEL_SHADOW_START_ANGLE,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// S_S_P
		FLOAT_STR_PARAMETER(SunShadowStartAngleDegrees,		PARAMETER_SUN_SHADOW_START_ANGLE,		FL_TRACKING_PARAMETER | FL_FLASH_STORAGE | FL_FLASH_UPDATE),	// S_S_S
	#endif	// USE_BACKTRACKING

	// RTCC
	DEC_STR_PARAMETER(Seconds,				PARAMETER_RTCC_SECONDS,				FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Minutes,				PARAMETER_RTCC_MINUTES,				FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Hours,				PARAMETER_RTCC_HOURS,				FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Day,					PARAMETER_RTCC_DAY,					FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Date,					PARAMETER_RTCC_DATE,				FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Month,				PARAMETER_RTCC_MONTH,				FL_SYSTEM_PARAMETER),
	DEC_STR_PARAMETER(Year,					PARAMETER_RTCC_YEAR,				FL_SYSTEM_PARAMETER),


	// Commands
	CMD_PARAMETER(Calibrate,				COMMAND_CALIBRATE,					FL_TRACKING_FUNCTION),
	CMD_PARAMETER(ForceCurrentOrientation,	COMMAND_FORCE_CURRENT_ORIENTATION,	FL_TRACKING_FUNCTION),
	CMD_PARAMETER(GetCurrentOrientation,	COMMAND_GET_CURRENT_ORIENTATION,	FL_TRACKING_FUNCTION),
	CMD_PARAMETER(GetSPAValues,				COMMAND_GET_SPA_VALUES, 			FL_TRACKING_FUNCTION),
	CMD_PARAMETER(ClearOrientation,			COMMAND_CLEAR_CURRENT_ORIENTATION,	FL_TRACKING_FUNCTION),
	CMD_PARAMETER(SetTrackingMode,			COMMAND_SET_TRACKING_MODE,			FL_SYSTEM_FUNCTION),



        CMD_PARAMETER(Run_Up,					COMMAND_RUN_UP,				FL_TRACKING_FUNCTION),
        CMD_PARAMETER(Run_Down,					COMMAND_RUN_DOWN,			FL_TRACKING_FUNCTION),
        CMD_PARAMETER(SMTALStow,                                COMMAND_SET_STOW_MODE,          	FL_TRACKING_FUNCTION),

        CMD_PARAMETER(SMTALMAN,                                 COMMAND_SET_MANUAL_MODE,		FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SMTALAUTO,                                COMMAND_SET_TRACKING_MODE,		FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SMTALStop,				COMMAND_STOP,				FL_TRACKING_FUNCTION),
        CMD_PARAMETER(SMTALSPATrak,				COMMAND_START,				FL_TRACKING_FUNCTION),
	CMD_PARAMETER(SMTALReset,				COMMAND_RESET,				FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SMTALRTCC,				COMMAND_RECV_RTCC,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(ES,				COMMAND_RUN_EAST,			FL_TRACKING_FUNCTION),
	CMD_PARAMETER(WE,				COMMAND_RUN_WEST,			FL_TRACKING_FUNCTION),
        CMD_PARAMETER(SW,                                COMMAND_SET_STOW_MODE,          	FL_TRACKING_FUNCTION),
        CMD_PARAMETER(WS,                                  COMMAND_SET_WIND_STOW,          	FL_TRACKING_FUNCTION),
        CMD_PARAMETER(WD,                                  COMMAND_SET_WIND_DISS,          	FL_TRACKING_FUNCTION),

        CMD_PARAMETER(MN,                                 COMMAND_SET_MANUAL_MODE,		FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(AU,                                COMMAND_SET_TRACKING_MODE,		FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SP,				COMMAND_STOP,				FL_TRACKING_FUNCTION),
        CMD_PARAMETER(TR,				COMMAND_START,				FL_TRACKING_FUNCTION),
        CMD_PARAMETER(SM,                         COMMAND_SET_MENU_MODE,			FL_SYSTEM_FUNCTION),
	CMD_PARAMETER(RS,				COMMAND_RESET,				FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(ST,				COMMAND_SEND_XBEE,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(SD,				COMMAND_ST_DAYS,				FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(RD,                 		COMMAND_RCV_DAYS,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(RT,				COMMAND_RECV_RTCC,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(HR,				COMMAND_RECV_Hour,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(MI,                     		COMMAND_RECV_Min,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SC,                                 COMMAND_RECV_Sec,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(DY,                                 COMMAND_RECV_Day,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(DT,                                COMMAND_RECV_Date,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(MT,                               COMMAND_RECV_Month,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(YR,                                COMMAND_RECV_Year,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(EN,         			COMMAND_RCV_ENV,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(LA,         			COMMAND_RCV_Lat,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(LO,         			COMMAND_RCV_Lon,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(AL,         			COMMAND_RCV_Alt,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(TZ,         			COMMAND_RCV_TZ,			        FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(SE,                 		COMMAND_RCV_STS,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(RL,                 		COMMAND_RCV_SLR,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(FL,                 		COMMAND_RCV_SLF,			FL_SYSTEM_FUNCTION),



        CMD_PARAMETER(BE,                 		COMMAND_BT_ENB,                         FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(BP,                 		COMMAND_BT_PANEL,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(BS,                 		COMMAND_BT_SUN,                         FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(RL,                 		COMMAND_RCV_SLR,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(FL,                 		COMMAND_RCV_SLF,			FL_SYSTEM_FUNCTION),

        CMD_PARAMETER(AT,                 		COMMAND_SEND_ATTR,			FL_SYSTEM_FUNCTION),
        CMD_PARAMETER(TE,				COMMAND_CALIBRATE,					FL_TRACKING_FUNCTION),
        // to be added:
	//		RF setup parameters?
	//		additional tracking parameters?
	//		additional device identification parameters?

	{
		NULL, NULL, 0, FL_NONE					// table terminator
	}
};
/////////////Xbee API
FILE_GLOBAL ARRAY char IdentityStr[10];
FILE_GLOBAL ARRAY char Coord_Addr[8];
FILE_GLOBAL ARRAY char Dest_Addr[8];
FILE_GLOBAL ARRAY char SFD[8];
FILE_GLOBAL ARRAY char Str[30];
FILE_GLOBAL UINT8 sti;
FILE_GLOBAL ARRAY char Message[200];
FILE_GLOBAL UINT8 len;
//*************************************************************************************************
//									Rx Message FSM
//*************************************************************************************************

void RxMessageFSM(UART_MODULE UARTid)
{
         UINT16 sum=0x0000; INT8 check_sum;
	PERSISTENT_LOCAL_INIT UINT8 bInputBufferIndex = 0;

    // *************************************************
    //		    State Transition
    // *************************************************
    // state transitions are based on:
    //
	//	    external events
    //		state complete
    //
    // state transitions handle setting up everything BEFORE entering the new state
    // if there is no need for a transition, we just stay in the current state

	// external events may indicate multiple, contradictory state transitions.
	// the order of processing of external events determines their absolute priority; the FIRST successfully processed transition will actually occur

	switch(stRxMsgState)
		{
		case ST_RX_MSG_INIT:				// initial state, only at power up
			stRxMsgState = ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR;
			fSubStateStatus = SUBSTATE_NOT_DONE;
                        bRxMsgFSMSequenceCtr = 0;
			//eSerialOutputMode = SER_MODE_MENU;
			break;

		case ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR:
			// collecting keystrokes and waiting for CR terminator
			// receieved keystroke(s) are processed in last substate, below
                        if (fSubStateStatus IS SUBSTATE_DONE)
				{
				if (bInputBufferIndex > 0)									// check for meaningful input
				{
					// sequence is complete, so bump state
					stRxMsgState = ST_RX_MSG_PROCESS_MSG;
				}
				// else stay in state, and just restart substate

				fSubStateStatus = SUBSTATE_NOT_DONE;
				bRxMsgFSMSequenceCtr = 0;
				}
			break;

		case ST_RX_MSG_PROCESS_MSG:
			// Process received message
			if (fSubStateStatus IS SUBSTATE_DONE)
				{
                                    if(stRxMsgServiceState IS ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR)
                                    {
                                        stRxMsgState = ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR;
                                        fSubStateStatus = SUBSTATE_NOT_DONE;
                                        bRxMsgFSMSequenceCtr = 0;
                                        stRxMsgServiceState = ST_RX_MSG_INIT;
                                        break;
                                    }
				// message processing is complete, so send response
				stRxMsgState = ST_RX_MSG_RESPONSE;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bRxMsgFSMSequenceCtr = 0;
				}

			// else not done with substates
			break;

		case ST_RX_MSG_WAIT_FOR_CMD:
			break;

		case ST_RX_MSG_RESPONSE:
			if (fSubStateStatus IS SUBSTATE_DONE)
				{
				// response has been sent, so bump state back to wait for another message
				stRxMsgState = ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR;
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bRxMsgFSMSequenceCtr = 0;
				}
			// else not done with substates
			break;


		#ifdef NOTDEF
			case ST_RX_MSG_DISPLAY_REALTIME_DATA:
				// check for keystroke received
				if ((fSubStateStatus IS SUBSTATE_DONE) AND (fSelection IS MENU_SELECTED))
					{
					// sequence is complete (keystroke received), so bump state back to re-display menu
					stRxMsgState = ST_RX_MSG_DISPLAY_MENU;

					fSubStateStatus = SUBSTATE_NOT_DONE;
					bRxMsgFSMSequenceCtr = 0;
					}
				// else not done with substates
				break;
		#endif

	    default:
            RuntimeError(RX_MSG_ERROR_INVALID_STATE);
            break;

		}		// switch(stRxMsgState)

    // *************************************************
    //		    Process the Current or New State
    // *************************************************
    // State handler
    //	called one or more times for a state, depending on state implementation
    //	state transitions do NOT occur here
    //	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
    //	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate

	switch(stRxMsgState)
		{
		case ST_RX_MSG_INIT:				// we should NEVER be in this state
            RuntimeError(RX_MSG_ERROR_INVALID_STATE);
			break;

		case ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR:				// ST_RX_MSG_WAIT_FOR_CR:
			// collecting keystrokes and waiting for CR terminator
			switch(bRxMsgFSMSequenceCtr)
				{
				case 0:
					// clear the input and return buffers
					pgcInputBuffer[0] = SZ_TERM;
					szReturnStr[0] = SZ_TERM;
					bInputBufferIndex = 0;						// initialize input buffer index (input should be more than one character)

					// start serial receive process
					// if there is already a byte available, skip StartSerialRx() so we do not lose it..
					if (AnySerialRxDataAvailable(UARTid) IS_FALSE)
						{
						IGNORE_RETURN_VALUE StartSerialRx(UARTid);
						}

					++bRxMsgFSMSequenceCtr;						// force next substate on next FSM entry
					break;

				case 1:
					// check for available keystroke ==> this is WRONG, does not handle end of buffer correctly!! (?)
					if ((AnySerialRxDataAvailable(UARTid) IS_TRUE) AND (bInputBufferIndex < INPUT_BUFFER_SIZE))
					{
						// a keystroke is available. read the keystroke and put it in the buffer
						pgcInputBuffer[bInputBufferIndex] = (char)ReadSerialRxdData(UARTid);

						#ifdef NOTDEF
							// this provides local echo, which is really only suitable when using a terminal emulator to simulate wireless communication
							if (isprint(pgcInputBuffer[bInputBufferIndex]) IS_NOT ZERO)
							{
								// character is printable, so display it - regardless of whether or not it is valid
								DisplayCharacter(UARTid, pgcInputBuffer[bInputBufferIndex], NO_WAIT_FOR_DISPLAY);		// echo character; we are assuming this will complete before additional message below
							}
						#endif

						// check for end of input
						/*if ((pgcInputBuffer[bInputBufferIndex] IS ASCII_CR) OR (pgcInputBuffer[bInputBufferIndex] IS ASCII_LF) OR (pgcInputBuffer[bInputBufferIndex] IS SZ_TERM))
						{
							// input is complete
							pgcInputBuffer[bInputBufferIndex] = SZ_TERM;			// terminate input string

							// check for non-zero length of input string (zero length string is interpreted as... ZERO)
							if (bInputBufferIndex > 0)
							{
								++bRxMsgFSMSequenceCtr;								// non-zero length string, so this must be the terminator for an entry; force next substate on next FSM entry for processing of parameter
							}
							else
							{
								fSubStateStatus = SUBSTATE_DONE;					// no input string, so we are done with the ENTIRE state
							}
						}
						// Check character for valid type; numeric data ONLY. Note processing sequence order; CR and ESC are non-numeric, too!
						else if (pgcInputBuffer[bInputBufferIndex] IS ASCII_ESC)
						{
							// abort substate
							fSubStateStatus = SUBSTATE_DONE;					// we are done with the ENTIRE state
							//++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry  <sek> 26 Feb 13
							break;
						}*/
                                                #ifdef DIGI
                                                if(pgcInputBuffer[bInputBufferIndex] == 0x7E)
                                                {
                                                    pgcInputBuffer[0] == 0x7E;
                                                    bInputBufferIndex = 0;
                                                }
                                                //checking zigbee length bytes
                                                if (bInputBufferIndex IS (pgcInputBuffer[2]+0x03))
						{
							// input is complete
							pgcInputBuffer[bInputBufferIndex+1] = SZ_TERM;			// terminate input string
                                                        //checksum
                                                        for(sti=3;sti<bInputBufferIndex;sti++)
                                                        {
                                                            sum = sum + pgcInputBuffer[sti];
                                                        }
                                                        check_sum = (0xFF-(sum&0x00FF));
                                                        //validating checksum
                                                        if(check_sum == pgcInputBuffer[bInputBufferIndex])
                                                        {
                                                            // Coordinate address
                                                            Coord_Addr[0] = pgcInputBuffer[4];
                                                            Coord_Addr[1] = pgcInputBuffer[5];
                                                            Coord_Addr[2] = pgcInputBuffer[6];
                                                            Coord_Addr[3] = pgcInputBuffer[7];
                                                            Coord_Addr[4] = pgcInputBuffer[8];
                                                            Coord_Addr[5] = pgcInputBuffer[9];
                                                            Coord_Addr[6] = pgcInputBuffer[10];
                                                            Coord_Addr[7] = pgcInputBuffer[11];
                                                            // Command
                                                            for(sti = 0;sti<=(bInputBufferIndex - 16);sti++)
                                                                Str[sti]= pgcInputBuffer[15+sti];
                                                            //Str[0] = pgcInputBuffer[15];
                                                            //Str[1] = pgcInputBuffer[16];
                                                            //Str[2] = SZ_TERM;
                                                            Str[sti] = SZ_TERM;
                                                        }
                                                        else
                                                        {
                                                            Str[0] = SZ_TERM;
                                                        }
							// check for non-zero length of input string (zero length string is interpreted as... ZERO)
							if (bInputBufferIndex > 0)
							{
								++bRxMsgFSMSequenceCtr;								// non-zero length string, so this must be the terminator for an entry; force next substate on next FSM entry for processing of parameter
							}
							else
							{
								fSubStateStatus = SUBSTATE_DONE;					// no input string, so we are done with the ENTIRE state
							}
                                                    // force next substate on next FSM entry for processing of parameter
						}
                                                else if (bInputBufferIndex IS INPUT_STR_SIZE)
						{
							// input buffer is full
							++bInputBufferIndex;								// bump index to point at terminator

							pgcInputBuffer[bInputBufferIndex] = SZ_TERM;		// terminate input string

							++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry for processing of parameter
						}
						else
						{
							// accept character and stay in state and substate to collect more characters
							++bInputBufferIndex;								// bump index
						}
						// anything printable is allowable here, due to different parameter data types
                                                #endif
                                                #ifdef CC2530     //02A879C30F 1111 2222 12 34089000


                                                if(recvflag == 0)
                                                {
                                                    if((pgcInputBuffer[bInputBufferIndex] IS 0x0F))// AND (pgcInputBuffer[(bInputBufferIndex - 1)] IS 0xC3))//&&(pgcInputBuffer[bInputBufferIndex - 2] IS 0x79)&&(pgcInputBuffer[bInputBufferIndex - 3] IS 0xA8)&&(pgcInputBuffer[bInputBufferIndex - 4] IS 0x02))
                                                    {       //01234 02 A8 79 C3 0F
                                                        SFD[4]= pgcInputBuffer[bInputBufferIndex];
                                                        SFD[3]= pgcInputBuffer[bInputBufferIndex - 1];
                                                        SFD[2]= pgcInputBuffer[bInputBufferIndex - 2];
                                                        SFD[1]= pgcInputBuffer[bInputBufferIndex - 3];
                                                        SFD[0]= pgcInputBuffer[bInputBufferIndex - 4];
                                                           if(Check_PKT_Format(SFD,5) IS TRUE)        //one to one msg
                                                           {
                                                                        pgcInputBuffer[bInputBufferIndex] = 0x0F;
                                                                        bInputBufferIndex = 5;
                                                                        chkflag = 5;
                                                                        recvflag = 1;
                                                           }
                                                           else if(Check_PKT1_Format(SFD,5) IS TRUE)    //broadcost msg
                                                           {
                                                                        pgcInputBuffer[bInputBufferIndex] = 0x0F;
                                                                        bInputBufferIndex = 5;
                                                                        chkflag = 5;
                                                                        recvflag = 1;
                                                           }
                                                           else
                                                               bInputBufferIndex = 0;
                                                    }
                                               
                                                else if (bInputBufferIndex IS INPUT_STR_SIZE)
						{
							// input buffer is full
							++bInputBufferIndex;								// bump index to point at terminator

							pgcInputBuffer[bInputBufferIndex] = SZ_TERM;		// terminate input string

							++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry for processing of parameter
						}
						else
						{
							// accept character and stay in state and substate to collect more characters
							++bInputBufferIndex;								// bump index
						}
                                                }

                                                else if(recvflag == 1){
                                                //checking zigbee length bytes
                                                    if((pgcInputBuffer[bInputBufferIndex] IS 0x4F))
                                                        recvflag = 0;
                                                if (bInputBufferIndex IS (pgcInputBuffer[9]))
						{
							// input is complete
							pgcInputBuffer[bInputBufferIndex+1] = SZ_TERM;			// terminate input string
                                                            xbee_addr[0] = pgcInputBuffer[5];
                                                            xbee_addr[1] = pgcInputBuffer[6];
                                                       

                                                            // Coordinate address
                                                            Coord_Addr[0] = pgcInputBuffer[7];
                                                            Coord_Addr[1] = pgcInputBuffer[8];

                                                            // Command
                                                            for(sti = 0;sti<=(bInputBufferIndex - 10);sti++)
                                                                Str[sti]= pgcInputBuffer[10+sti];
                                                            //Str[0] = pgcInputBuffer[15];
                                                            //Str[1] = pgcInputBuffer[16];
                                                            //Str[2] = SZ_TERM;
                                                            Str[sti] = SZ_TERM;
                                                            recvflag = 0;
//                                                        }
//                                                        else
//                                                        {
//                                                            ++bRxMsgFSMSequenceCtr;   //exit for next parameter
//                                                            break;
//                                                        }
							// check for non-zero length of input string (zero length string is interpreted as... ZERO)
							if (bInputBufferIndex > 0)
							{
								++bRxMsgFSMSequenceCtr;								// non-zero length string, so this must be the terminator for an entry; force next substate on next FSM entry for processing of parameter
                                                           //     bInputBufferIndex = 0;
							}
							else
							{
								fSubStateStatus = SUBSTATE_DONE;					// no input string, so we are done with the ENTIRE state
							}
                                                    // force next substate on next FSM entry for processing of parameter
						}
                                                
						// anything printable is allowable here, due to different parameter data types
                                                
						// check for buffer overrun
						else if (bInputBufferIndex IS INPUT_STR_SIZE)
						{
							// input buffer is full
							++bInputBufferIndex;								// bump index to point at terminator

							pgcInputBuffer[bInputBufferIndex] = SZ_TERM;		// terminate input string

							++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry for processing of parameter
						}
						else
						{
							// accept character and stay in state and substate to collect more characters
							++bInputBufferIndex;								// bump index
						}
                                                
                                                }
                                                #endif
					}
					// No character available, stay in state and substate to collect more characters
					break;

				case 2:
					if (bInputBufferIndex > 0)									// <sek> 22 Sep 13 check for meaningful input
					{
						fSubStateStatus = SUBSTATE_DONE;						// we are done with the state
					}

					break;

				default:
					RuntimeError(RX_MSG_ERROR_INVALID_SUBSTATE);
					fSubStateStatus = SUBSTATE_DONE;
					break;
			}
			break;

		case ST_RX_MSG_PROCESS_MSG:
			// Process received message
			switch(bRxMsgFSMSequenceCtr)
				{
				case 0:
                                       IGNORE_RETURN_VALUE RxMessageHandler(&Str[0]);
                                            ++bRxMsgFSMSequenceCtr;
                                       /* if((strstr(pgcInputBuffer,IdentityStr))OR(strstr(pgcInputBuffer,"SMTAL")))
                                        {
                                            IGNORE_RETURN_VALUE RxMessageHandler(&pgcInputBuffer[0]);
                                            ++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry to wait for completion of command
                                        }
                                        else
                                        {
                                            stRxMsgServiceState = ST_RX_MSG_WAIT_FOR_PARAMETER_AND_CR;
                                            ++bRxMsgFSMSequenceCtr;
                                        }*/
                                        break;

				case 1:
                                        Str[0] = SZ_TERM;            ///<nn> Is it nessesary
                                        recvflag = 0;

					++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry for processing of parameter
					fSubStateStatus = SUBSTATE_DONE;
					break;

				default:
					RuntimeError(RX_MSG_ERROR_INVALID_SUBSTATE);
					fSubStateStatus = SUBSTATE_DONE;
					break;

				}
			break;

		case ST_RX_MSG_WAIT_FOR_CMD:
			fSubStateStatus = SUBSTATE_DONE;
			break;

		case ST_RX_MSG_RESPONSE:
			switch(bRxMsgFSMSequenceCtr)
				{
				case 0:
					//StartTransmitString(UARTid, szReturnStr);
                                        TXMode[UARTid] = 1;
                                        SendMBMessage(UARTid, szReturnStr,SetTXLen[UARTid]+1, WAIT_FOR_DISPLAY);

					++bRxMsgFSMSequenceCtr;								// force next substate on next FSM entry to wait for completion of transmit
					break;

				case 1:
					// substate to check for menu display (RS-232 transmission) complete
					if (IsTransmitComplete(UARTid) IS_TRUE)
						{TXMode[UARTid] = 0;
							SetTXLen[UARTid] = 0;
						++bRxMsgFSMSequenceCtr;						// force next substate on next FSM entry, actually an invalid state
						fSubStateStatus = SUBSTATE_DONE;
						}
					// else stay in this state until display of line is complete
					break;

				default:
					RuntimeError(RX_MSG_ERROR_INVALID_SUBSTATE);
					fSubStateStatus = SUBSTATE_DONE;
					break;

				}
			break;


		#ifdef NOTDEF
			case ST_RX_MSG_DISPLAY_REALTIME_DATA:
				switch(bRxMsgFSMSequenceCtr)
					{
					case 0:
						// enable realtime serial data display
						bRealTimeDisplay = TRUE;
						eSerialOutputMode = SER_MODE_REALTIME;

						// start serial receive process
						// if there is already a byte available, skip StartSerialRx() so we do not lose it..
						if (AnySerialRxDataAvailable(UARTid) IS_FALSE)
							{
							IGNORE_RETURN_VALUE StartSerialRx(UARTid);
							}
						++bRxMsgFSMSequenceCtr;						// force next substate on next FSM entry
						break;

					case 1:
						// check for available keystroke
						if (AnySerialRxDataAvailable(UARTid) IS_TRUE)
							{
							// a keystroke is available
							pgcInputBuffer[0] = (char)ReadSerialRxdData(UARTid);
							fSelection = MENU_SELECTED;

							// any keystroke ends the Realtime Data streaming and returns to the menu
							// disable realtime serial data display
							bRealTimeDisplay = FALSE;
							eSerialOutputMode = SER_MODE_MENU;

							// we may want to add another substate here, to wait for any pending display to complete

							++bRxMsgFSMSequenceCtr;						// force next (invalid) substate on next FSM entry
							fSubStateStatus = SUBSTATE_DONE;			// keystroke or not, we are done with the state
							}
						else
							{
							fSelection = NOTHING_SELECTED;			// no selection made

							// stay in state and substate
							}
						break;
				}
			#endif

			default:
				RuntimeError(RX_MSG_ERROR_INVALID_SUBSTATE);
				fSubStateStatus = SUBSTATE_DONE;
				break;
		}


	// Note that we do not check for unprocessed events here.. because the FSM is not setup to handle externally generated events.

}


//*************************************************************************************************
//								Remote Command Processors
//*************************************************************************************************

// find the structure in the jump table with the name that matches the received message
// Jump Table function pointers dispatch to:
//		ParameterHandler()
//		CommandHandler()

// Caller makes sure that the buffer at ptrBuffer ends with a SZ_TERM

// ParameterHandler() determines if the message is to set a value (value follows parameter name) or return a value (nothing follows parameter name)
// CommandHandler()

BOOL RxMessageHandler(char* ptrBuffer)
{
	jumpTable_t *ptrTable = &MsgJumpTable[0];						// point at start of the jump table
	UINT8 tblIndex;
	size_t NameLen;

	/* Loop through the Jump Table to find a matching Parameter or Command name string */
	for (tblIndex = 0; ptrTable->pstrMsgName != NULL; tblIndex++, ptrTable++)
	{
		NameLen = strlen(ptrTable->pstrMsgName);					// length of parameter or command name

		// Do a length limited match from the buffer against Parameter and Command names in the jump table.
		if (strnicmp((char*) ptrBuffer, (char*) ptrTable->pstrMsgName, NameLen) == 0)
		{

			// **********************************
			//	Call Command thru Jump Table
			// **********************************
			// Increment the buf pointer (perhaps not a good idea?)
			// appropriately before calling the jump table function.
//			(ptrTable->ptrFunction)(ptrBuffer + NameLen + 1, tblIndex);
			(ptrTable->ptrFunction)(ptrBuffer + NameLen, tblIndex);

			// collect attribute flags for subsequent processing
			//fgcAttributeFlags |= ptrTable->flags;

			return TRUE;
		}
	}
	// No match for Parameter or Command Name found.
	*ptrBuffer = SZ_TERM;					// terminate the input buffer, to remove any discarded input
	RuntimeError(RX_MSG_ERROR_INVALID_PARAMETER);
        strcpy(szReturnStr, " ");
        #ifdef USE_MOVE_SEQ_FSM_STEP_VERBOSE
        strcpy(szReturnStr, "\n\rERR\n\r");
        #endif
	return FALSE;
}


BOOL ParameterHandler(char *ptrBuffer, UINT8 bTableIndex)
{
	char *ptrValue = ptrBuffer;								// ptr to first character of Parameter value, if any
	UINT8 bufferIndex = 0;

	// input buffer contains either:
	//		parameter value to set
	//		nothing, request to return value

	// check length of input buffer, ignoring spaces. Buffer Terminator will be SZ_TERM (0x00), but there may also be CR, LF present
	while(*(ptrBuffer + bufferIndex) IS_NOT SZ_TERM)
	{
		if (*(ptrBuffer + bufferIndex) IS ASCII_SPACE)
		{
			// skip leading spaces
			++ptrValue;
			++bufferIndex;
		}
		else
		{
			break;					// not a terminator, not a space, so this must be a parameter value
		}
	}

	// initialize the return string with the Parameter name
	strcpy(szReturnStr, "\n\r");
	strcat(szReturnStr, (char*) MsgJumpTable[bTableIndex].pstrMsgName);
	strcat(szReturnStr, ": ");

	// check remaining length of input buffer.
	if (strlen(ptrValue) IS ZERO)
	{
		// No value present, so this is a request to return a value
		if (ReturnParameter(MsgJumpTable[bTableIndex].eParameter, szReturnStr + strlen(szReturnStr)) IS_TRUE)
		{
			strcat(szReturnStr, "\n\r");
			return TRUE;
		}
	}
	else
	{
		// value string present, so this is a request to set a value
		if (UpdateParameter(MsgJumpTable[bTableIndex].eParameter, ptrValue) IS_TRUE)
		{
			// check for required table update (make value persistent)
			if((MsgJumpTable[bTableIndex].flags  & FL_FLASH_UPDATE) IS FL_FLASH_UPDATE)
			{
				// write out modified flash parameters
				WriteFlashParameterTable();		// ==> should have a return value
			}
			strcat(szReturnStr, " OK\n\r");
			return TRUE;
		}
	}

	strcat(szReturnStr, " ERR\n\r");
	return FALSE;

}

BOOL CommandHandler(char *ptrBuffer, UINT8 bTableIndex)
{
	RTCC_DATE_TIME CurrentDateTime;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
	LOCAL SmartTrakOrientation SPAOrientation;
        LOCAL char temp[20];
        int nStatus = 0,i,tmpi; float tmpf;
	strcpy(szReturnStr, ""); int year,rdays;
        UINT8 hr,min,sec,dy,dt,mn,yr; BYTE ss;
        char *tkns[20];
	//strcat(szReturnStr, (char*) MsgJumpTable[bTableIndex].pstrMsgName);

	switch((enum tagCommand)MsgJumpTable[bTableIndex].eParameter)
	{

	// Commands
		case COMMAND_RUN_UP:
			#ifdef USE_ELEVATION
				if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
				{
					BITSET(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
					BITSET(efSwitchEvents, EF_SW_UP_SWITCH_OPEN_EVENT);				// OPEN will be processed AFTER CLOSED, has the effect of push and release
					strcat(szReturnStr, " OK\n\r");
				}
			#endif
			break;

		case COMMAND_RUN_EAST:
			#ifdef USE_AZIMUTH
				if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
				{
                                    if((MAN_WEST IS 0)AND (MAN_EAST IS 0)AND (MAN_STOW IS 0))
                                    {
                                        MAN_EAST = 1;
					BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
					BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
					strcpy(szReturnStr, "East\n\r");
                                    }
				}
                                status();
			#endif
			break;

		case COMMAND_RUN_WEST:
			#ifdef USE_AZIMUTH
				if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
				{
                                    if((MAN_WEST IS 0)AND (MAN_EAST IS 0)AND (MAN_STOW IS 0))
                                    {
                                        MAN_WEST = 1;
					BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
					BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
					strcpy(szReturnStr, "West\n\r");
                                    }
				}
                                status();
			#endif
			break;

		case COMMAND_RUN_DOWN:
			#ifdef USE_ELEVATION
				if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
				{
					BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
					BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
					strcat(szReturnStr, " OK\n\r");
				}
			#endif
			break;
                case COMMAND_SET_STOW_MODE:
                            {
                                #ifdef USE_AZIMUTH
				if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
				{
                                    if((MAN_WEST IS 0)AND (MAN_EAST IS 0) AND (MAN_STOW IS 0))
                                    {
                                        MAN_STOW = 1;
					BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
					BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
					strcpy(szReturnStr, "STOW\n\r");
                                    }
				}
                                #endif
                            }
                            status();

                        break;

                    case COMMAND_SET_WIND_STOW:
                        if (ptrRAM_SystemParameters->ucTracking_Mode IS_NOT MODE_MANUAL)
                        {
                        strcpy(szReturnStr, "WIND STOW\n\r");
                        BITSET(efPanelPositionEvents, EF_PANEL_POS_WIND_STOW);
                        BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_WIND_STOW);
                        }

                    break;

                    case COMMAND_SET_WIND_DISS:
                        if (ptrRAM_SystemParameters->ucTracking_Mode IS_NOT MODE_MANUAL)
                        {
                        strcpy(szReturnStr, "WIND STOW DISS\n\r");
                        BITSET(efPanelPositionEvents, EF_PANEL_POS_END_WIND_STOW);
                        BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_WIND_STOW);
                        }
                    break;



		case COMMAND_STOP:
			// make sure there is an active motor to stop
			if (IsMoveSequenceComplete() IS_FALSE)
			{
                                MAN_WEST = MAN_EAST = MAN_STOW = 0;
				// there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
				BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);		// CLOSED is processed first, same event as button push
				strcpy(szReturnStr, "Stop\n\r");
				// no need for an OPEN even, STOP clears all switch and virtual switch events
			}

			break;

                case COMMAND_START:
                        SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
			BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
                        BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);		// OPEN will be processed AFTER CLOSED, has the effect of push and release
                        strcpy(szReturnStr, "Trak\n\r");
                    break;

                case COMMAND_SET_SERVICE_MODE:
                        if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
                        {
                            ptrRAM_SystemParameters->ucTracking_Mode = MODE_MANUAL;
                            strcpy(szReturnStr, "Manual\n\r");
                        }
                        else
                        {
                            ptrRAM_SystemParameters->ucTracking_Mode = MODE_TRACKING;
                            strcpy(szReturnStr, "Auto\n\r");
                        }
                        WriteFlashParameterTable();

			break;

		case COMMAND_CALIBRATE:
                    if (ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
                    {
			BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_OPEN_EVENT);
			strcat(szReturnStr, " OK\n\r");
                    }
			break;

		case COMMAND_SET_TRACKING_MODE:
			ptrRAM_SystemParameters->ucTracking_Mode = MODE_TRACKING;
                        WriteFlashParameterTable();
                        ClearCommandStarted(MOTOR_AZIMUTH);
                        Finish_MotionStats(MOTOR_AZIMUTH);
                        MAN_WEST = MAN_EAST = MAN_STOW = 0;
                        SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
			BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_CLOSED_EVENT);	// start with virtual button press for ButtonProcessingFSM() handling
                        BITSET(efVirtualSwitchEvents, EF_VSW_SPA_TRACK_SWITCH_OPEN_EVENT);
			status();
			break;

                case COMMAND_SET_MANUAL_MODE:
			ptrRAM_SystemParameters->ucTracking_Mode = MODE_MANUAL;
                        WriteFlashParameterTable();
                        MAN_WEST = MAN_EAST = MAN_STOW = 0;
                        //BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);
                        ClearCommandStarted(MOTOR_AZIMUTH);
                        Finish_MotionStats(MOTOR_AZIMUTH);							// mark command as started so we cannot misinterpret completion
                        BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_STOP);	// bring to an orderly stop
                        BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);
                        ResetMotionFSM();
			status();
			break;

               

		case COMMAND_RESET:
			BITSET(efSwitchEvents, EF_SW_RESET_SWITCH_CLOSED_EVENT);
			strcat(szReturnStr, " OK\n\r");
			break;

                  case COMMAND_ST_DAYS:

                        break;

                  case COMMAND_SET_REMOTE_MODE:
			ptrRAM_SystemParameters->eSerialOutputMode = SER_MODE_REMOTE;
			eSerialOutputMode = SER_MODE_REMOTE;								// update program global copy
			//ChangeSerialBaudRate(SERIAL_MENU_UART, DESIRED_MENU_BAUDRATE);
                        WriteFlashParameterTable();
			strcat(szReturnStr, " OK\n\r");
			break;

		case COMMAND_SET_MENU_MODE:
			ptrRAM_SystemParameters->eSerialOutputMode = SER_MODE_MENU;
			eSerialOutputMode = SER_MODE_MENU;								// update program global copy
			ChangeSerialBaudRate(SERIAL_MENU_UART, DESIRED_MENU_BAUDRATE);
                        WriteFlashParameterTable();
			strcat(szReturnStr, " OK\n\r");
			break;
                case COMMAND_FORCE_CURRENT_ORIENTATION:
			// this is implemented by calculating the offset from the Mechanical Orientation to the SPA Orientation

			// get current date and time

                    if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
			{
				RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
				strcat(szReturnStr, " ERR\n\r");
				return FALSE;
			}
			IGNORE_RETURN_VALUE FormatRTCCDateTime(szReturnStr, ptrDateTime);

			// calculate and display Sun Position, global coordinates
			/*fgSPAMove.SunPositionState = */ IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
			strcat(szReturnStr, " SPA: ");
			Orientation_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);	// format for display

			// convert SPA orientation to local orientation
			ConvertSPAtoLocalOrientation(&SPAOrientation);

			// display new Local orientation
			strcat(szReturnStr, "Local: ");
			Orientation_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);	// format for display

			// ==> conversion to backtrack values seems to make NO sense here.. or does it?

			// set the MSI Tick counters for the new orientation
			// the Sun Position is calculated in Degrees, so we have to convert it to Ticks
			// ==>> should this treat the difference between SPA and current orientation as a new offset?
			CurrentPosition_Set(MOTOR_AZIMUTH, ConvertDegreesToMSITicks(SPAOrientation.fAzimuth, AXIS_AZIMUTH));
			CurrentPosition_Set(MOTOR_ELEVATION, ConvertDegreesToMSITicks(SPAOrientation.fElevation, AXIS_ELEVATION));

			// update the MCU RAM copy of the Current Orientation
			ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth = CurrentPosition_Read(MOTOR_AZIMUTH);
			ptrRTCC_RAM_MechanicalOrientation->lLastElevation = CurrentPosition_Read(MOTOR_ELEVATION);

			#ifdef USE_DS3232_RTCC
				// write MCU RAM copy of Current Orientation to NV RTCC RAM, so we can recover it if power is lost
				IGNORE_RETURN_VALUE UpdateRTCCRAMOrientation();
			#endif

			strcat(szReturnStr + strlen(szReturnStr), " \n\r");
			break;


		case COMMAND_GET_CURRENT_ORIENTATION:
			#ifdef USE_MMA8452Q_INCLINOMETER
				// re-Initialize Inclinometer Averaging by refilling the averaging array (twice).
				if (Init_InclinometerSampleAveraging() IS_NOT TRUE)
				{
					RuntimeError(RX_MSG_ERROR_INCL_READ_ERROR);
					strcat(szReturnStr + strlen(szReturnStr), " ERR\n\r");
					return FALSE;
				}
			#endif	//  USE_MMA8452Q_INCLINOMETER

			strcat(szReturnStr, " Local: ");
			CurrentLocalOrientation_Format(szReturnStr + strlen(szReturnStr));
			strcat(szReturnStr, "\n\r");
			break;

		case COMMAND_GET_SPA_VALUES:
			// calculate and return Sun Position, global coordinates
			// get current date and time
			if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
			{
				RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
				strcat(szReturnStr, " ERR\n\r");
				return FALSE;
			}
			IGNORE_RETURN_VALUE FormatRTCCDateTime(szReturnStr, ptrDateTime);

			strcat(szReturnStr, " SPA: ");
			IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
			SPA_Backtrack_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);

			// convert SPA orientation to local orientation
			ConvertSPAtoLocalOrientation(&SPAOrientation);

			// new Local orientation
			strcat(szReturnStr, " Local: ");
			Orientation_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);	// format for display

			// orientation adjusted for Backtracking
			IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
			strcat(szReturnStr, " Backtrk: ");
			Orientation_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);	// format for display

			//  adjusted SetPoint orientation
			IGNORE_RETURN_VALUE PanelPositionFSM(&SPAOrientation);
			strcat(szReturnStr, " SetPt: ");
			Orientation_Format(szReturnStr + strlen(szReturnStr), &SPAOrientation);	// format for display
			strcat(szReturnStr, "\n\r");
			break;

		case COMMAND_CLEAR_CURRENT_ORIENTATION:
			// Clear Current Orientation from RTCC NV RAM and MCU RAM Copy
			CurrentPosition_Clear(MOTOR_AZIMUTH);								// clear the MSI Tick counters
			CurrentPosition_Clear(MOTOR_ELEVATION);
			// ClearRTCCRAMOrientation() handles #ifdef USE_DS3232_RTCC internally
			IGNORE_RETURN_VALUE ClearRTCCRAMOrientation();						// clear MCU RAM copy of Orientation, then Update the RTCC NV RAM copy of Orientation
			strcat(szReturnStr, " OK\n\r");
			break;
                case COMMAND_RCV_STS:
                        strcpy(temp,ptrBuffer);

                        i=0;
                            tkns[i] = strtok(temp, ":");
                        while(tkns[i])
                            tkns[++i] = strtok(NULL, ":");
                        if(i != 4)
                            break;

                        sscanf(tkns[0], "%f",  &tmpf);
                        if ((tmpf >= AZ_SOFT_LIMIT_DEGREES_REVERSE) AND (tmpf <= 0))
                        {
                                ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse = tmpf;
                        }
                        sscanf(tkns[1], "%f",  &tmpf);
                        if ((tmpf >= 0) AND (tmpf <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
                        {
                                ptrRAM_SystemParameters->fSingle_SoftLimit_Forward = tmpf;
                        }
                        sscanf(tkns[2], "%d",  &tmpi);
                        if ((tmpi IS 0) OR (tmpi IS 1))
                        {
                                ptrRAM_SystemParameters->bBacktrackingEnabled = tmpi;
                        }
                        sscanf(tkns[3], "%f",  &tmpf);
                        if ((tmpf >= 0) AND (tmpf <= 90))
			{
				ptrRAM_SystemParameters->fSunShadowStartAngleDegrees = tmpf;
			}
                        
                        WriteFlashParameterTable();
                        strcat(szReturnStr, " OK\n\r");
                        break;

                case COMMAND_BT_ENB:
                        strcpy(temp,ptrBuffer);
                        //strcat(szReturnStr, ptrBuffer);
                        sscanf(temp, "%d",  &tmpi);
                        
                        // bounds check
                        if ((tmpi IS 0) OR (tmpi IS 1))
                                {
                                ptrRAM_SystemParameters->bBacktrackingEnabled = tmpi;
                                WriteFlashParameterTable();
                                }
                        else
                                {
                                strcat(szReturnStr, " Invalid Arguments\n\r");
                                    return FALSE;							// not a valid entry
                                }
                        break;
                        attributes();
                        break;

                case COMMAND_BT_PANEL:
                        strcpy(temp,ptrBuffer);
                        //strcat(szReturnStr, ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= AZ_SOFT_LIMIT_DEGREES_REVERSE) AND (tmpf <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
			{
				ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Arguments\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;
                case COMMAND_BT_SUN:
                        strcpy(temp,ptrBuffer);
                        //strcat(szReturnStr, ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= 0) AND (tmpf <= 90))
			{
				ptrRAM_SystemParameters->fSunShadowStartAngleDegrees = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Arguments\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;
                case COMMAND_RCV_SLR:
                        strcpy(temp,ptrBuffer);
                        //strcat(szReturnStr, ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= AZ_SOFT_LIMIT_DEGREES_REVERSE) AND (tmpf <= 0))
			{
				ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Arguments\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;



                 case COMMAND_RCV_SLF:
                        strcpy(temp,ptrBuffer);
                        //strcat(szReturnStr, ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= 0) AND (tmpf <= AZ_SOFT_LIMIT_DEGREES_FORWARD))
			{
				ptrRAM_SystemParameters->fSingle_SoftLimit_Forward = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Arguments\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;

                case COMMAND_RCV_ENV:

                        strcpy(temp,ptrBuffer);

                        i=0;
                            tkns[i] = strtok(temp, ":");
                        while(tkns[i])
                            tkns[++i] = strtok(NULL, ":");
                        if(i != 4)
                            break;
                        sscanf(tkns[0], "%f",  &tmpf);

                        if ((tmpf >= MIN_LATITUDE) AND (tmpf <= MAX_LATITUDE))
				{
				ptrRAM_SystemParameters->fLatitude = tmpf;
				}
                        sscanf(tkns[1], "%f",  &tmpf);
                        if ((tmpf >= MIN_LONGITUDE) AND (tmpf <= MAX_LONGITUDE))
				{
				ptrRAM_SystemParameters->fLongitude = tmpf;
				}
                        sscanf(tkns[2], "%f",  &tmpf);
                        if ((tmpf >= MIN_ALTITUDE) AND (tmpf <= MAX_ALTITUDE))
				{
				ptrRAM_SystemParameters->fAltitude = tmpf;
				}
                        sscanf(tkns[3], "%f",  &tmpf);
                        if ((tmpf >= MIN_TIMEZONE) AND (tmpf <= MAX_TIMEZONE))
				{
				ptrRAM_SystemParameters->fTimeZone = tmpf;
				}

                        WriteFlashParameterTable();
                        status();
                        break;

                case COMMAND_RCV_Lat:

                        strcpy(temp,ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= MIN_LATITUDE) AND (tmpf <= MAX_LATITUDE))
			{
				ptrRAM_SystemParameters->fLatitude = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Argument\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;

                 case COMMAND_RCV_Lon:

                        strcpy(temp,ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= MIN_LONGITUDE) AND (tmpf <= MAX_LONGITUDE))
			{
				ptrRAM_SystemParameters->fLongitude = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Argument\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;

                  case COMMAND_RCV_Alt:

                        strcpy(temp,ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= MIN_ALTITUDE) AND (tmpf <= MAX_ALTITUDE))
			{
				ptrRAM_SystemParameters->fAltitude = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Argument\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;

                case COMMAND_RCV_TZ:

                        strcpy(temp,ptrBuffer);
                        sscanf(temp, "%f",  &tmpf);
                        if ((tmpf >= MIN_TIMEZONE) AND (tmpf <= MAX_TIMEZONE))
			{
				ptrRAM_SystemParameters->fTimeZone = tmpf;
                                WriteFlashParameterTable();
			}
                        else
                        {
                            strcat(szReturnStr, " Invalid Argument\n\r");
                            return FALSE;
                        }
                        attributes();
                        break;


                case COMMAND_SEND_ATTR:

                       // strcpy(szReturnStr,IdentityStr);
                        //strcat(szReturnStr, ":");
                        strcpy(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLatitude, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLongitude, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAltitude, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fTimeZone, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2((float)ptrRAM_SystemParameters->bBacktrackingEnabled, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSunShadowStartAngleDegrees, &nStatus));
                        strcat(szReturnStr, ":");
//                       strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward, &nStatus));
//                        strcat(szReturnStr, ":");
//                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse, &nStatus));
//                        strcat(szReturnStr, ":");
//                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Forward, &nStatus));
//                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse, &nStatus));
                        strcat(szReturnStr, ":");
                        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Forward, &nStatus));
                        strcat(szReturnStr, "\n\r");
                        strcpy(Message,szReturnStr);
                        len = strlen(Message);
                        Xbee_Frame(szReturnStr,Message,len,Coord_Addr);
                        break;

                case COMMAND_RECV_RTCC:

                        strcpy(temp,ptrBuffer);

                        i=0;
                            tkns[i] = strtok(temp, ":");
                        while(tkns[i])
                            tkns[++i] = strtok(NULL, ":");
                        if(i != 6)
                            break;

                        hr  = (UINT8)atoi(tkns[0]);
                        min = (UINT8)atoi(tkns[1]);
                        dy  = (UINT8)atoi(tkns[2]);
                        dt  = (UINT8)atoi(tkns[3]);
                        mn  = (UINT8)atoi(tkns[4]);
                        year  = atoi(tkns[5]);
                        yr    = (UINT8)(year - 2000);
                        if ((hr >= DS3232_MIN_HOURS_24) AND (hr <= DS3232_MAX_HOURS_24))
                        {
                            hr = BYTEtoBCD(hr);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_HOURS, hr);
			} for(i=0;i<=10;i++);
                        if ((min >= DS3232_MIN_MINUTES) AND (min <= DS3232_MAX_MINUTES))
                        {
                            min = BYTEtoBCD(min);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_MINUTES, min);
			}for(i=0;i<=10;i++);
                        if ((dy >= DS3232_MIN_DAY) AND (dy <= DS3232_MAX_DAY))
                        {
                            dy = BYTEtoBCD(dy);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_DAY, dy);
			}for(i=0;i<=10;i++);
                        if ((dt >= DS3232_MIN_DATE) AND (dt <= DS3232_MAX_DATE))
                        {
                            dt = BYTEtoBCD(dt);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_DATE, dt);
			}for(i=0;i<=10;i++);
                        if ((mn >= DS3232_MIN_MONTH) AND (mn <= DS3232_MAX_MONTH))
                        {
                            mn = BYTEtoBCD(mn);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_MONTH, mn);
			}for(i=0;i<=10;i++);
                        if ((yr >= DS3232_MIN_YEAR) AND (yr <= DS3232_MAX_YEAR))
                        {
                            yr = BYTEtoBCD(yr);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_YEAR, yr);
			}
                        status();
                        strcpy(szReturnStr,"Done\n\r");
                        break;

                case COMMAND_RECV_Hour:
                       hr  = (UINT8)atoi(ptrBuffer);
                       if ((hr >= DS3232_MIN_HOURS_24) AND (hr <= DS3232_MAX_HOURS_24))
                       {
                            hr = BYTEtoBCD(hr);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_HOURS, hr);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;

                case COMMAND_RECV_Min:
                       min  = (UINT8)atoi(ptrBuffer);
                       if ((min >= DS3232_MIN_SECONDS) AND (min <= DS3232_MAX_SECONDS))
                       {
                            min = BYTEtoBCD(min);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_MINUTES, min);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;

                   case COMMAND_RECV_Sec:
                       sec  = (UINT8)atoi(ptrBuffer);
                       if ((sec >= DS3232_MIN_SECONDS) AND (sec <= DS3232_MAX_SECONDS))
                       {
                            sec = BYTEtoBCD(sec);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_HOURS, sec);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;

                 case COMMAND_RECV_Day:
                       dy  = (UINT8)atoi(ptrBuffer);
                       if ((dy >= DS3232_MIN_DAY) AND (dy <= DS3232_MAX_DAY))
                       {
                            dy = BYTEtoBCD(dy);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_DAY, dy);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;

                 case COMMAND_RECV_Date:
                       dt  = (UINT8)atoi(ptrBuffer);
                       if ((dt >= DS3232_MIN_DATE) AND (dt <= DS3232_MAX_DATE))
                       {
                            dt = BYTEtoBCD(dt);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_DATE, dt);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;

                case COMMAND_RECV_Month:
                       mn  = (UINT8)atoi(ptrBuffer);
                       if ((mn >= DS3232_MIN_MONTH) AND (mn <= DS3232_MAX_MONTH))
                       {
                            mn = BYTEtoBCD(mn);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_MONTH, mn);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;
               case COMMAND_RECV_Year:
                       year  = (UINT8)atoi(ptrBuffer);
                       yr    = (UINT8)(year - 2000);
                       if ((yr >= DS3232_MIN_YEAR) AND (yr <= DS3232_MAX_YEAR))
                       {
                            yr = BYTEtoBCD(yr);		// convert UINT8 to BCD
                            WriteRTCCRegister(DS3232_REG_YEAR, yr);
		       }
                       else
                       {
                           strcpy(szReturnStr,"Invalid Number\n\r");
                           return FALSE;
                       }
                       status();
                       strcpy(szReturnStr,"Done\n\r");
                    break;
                case  COMMAND_RCV_DAYS:
                       rdays = (UINT8)atoi(ptrBuffer);
                       ptrRAM_SystemParameters->fSingle_stop_days = rdays;
                       WriteFlashParameterTable();
                       break;
                case COMMAND_SEND_XBEE:
                        status();
                        /*if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
			{
				RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
				strcpy(szReturnStr, "0:0:0:0:0:0:0:0:0:0\n\r");
                                strcpy(Message,szReturnStr);
                                len = strlen(Message);
                                Xbee_Frame(szReturnStr,Message,len,Coord_Addr);
				return FALSE;
			}
                        else
                        {
                            IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr, ptrDateTime);
                        }
                        strcat(szReturnStr, ":");
                        IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
                        ConvertSPAtoLocalOrientation(&SPAOrientation);

                        strcat(szReturnStr,(const char *)ftoa2(SPAOrientation.fAzimuth, &nStatus));
                        strcat(szReturnStr, ":");

                        if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
                        {
                            IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
                            ss = PanelPositionFSMM(&SPAOrientation);

                            strcat(szReturnStr,(const char *)ftoa(SPAOrientation.fAzimuth , &nStatus));
                            strcat(szReturnStr, ":");
                        }
                        else
                        {
                            strcat(szReturnStr,(const char *)ftoa(pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
                            strcat(szReturnStr, ":");
                        }

                        #ifdef USE_MMA8452Q_INCLINOMETER
			   // strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
                           // strcat(szReturnStr, ":");

                            strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
                            strcat(szReturnStr, ":");
			#endif

                        //IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr + strlen(szReturnStr), ptrDateTime);
                        //strcat(szReturnStr, ":");
                        //4digits    x1x2x3x4
                        //    x1- 0POWER_UP/1INIT/2TRACKING/3HOLD/4NIGHT_STO/5WIND_STOW
                        //    x2- 1MAN/2AUTO     x3- 0stopped/1West/2East
                        //    x4- 0MotorStooped/1Motor running
                        BYTEtoASCIIstr(ss, temp);
                        if(((int)ss < 1) || ((int)ss > 5))
                             strcat(szReturnStr,"06");
                        else
                             strcat(szReturnStr, temp);
                        if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
                        {
                            if(MAN_EAST == 1)
                                strcat(szReturnStr, "12");
                            else if(MAN_WEST == 1)
                                strcat(szReturnStr, "11");  //West
                            else if(MAN_STOW == 1)
                                strcat(szReturnStr, "13");  //West
                            else
                                strcat(szReturnStr, "10");
                        }
                        else
                        {
                            if(bTrackerDirection == 1)
                                strcat(szReturnStr, "21");  //West
                            else if(bTrackerDirection == 2)
                                strcat(szReturnStr, "22");
                            else
                                strcat(szReturnStr, "20");
                        }
                        
                        if (IsCommandComplete(AXIS_AZIMUTH) IS_TRUE)
                            strcat(szReturnStr, "0\n\r");
                        else
                            strcat(szReturnStr, "1\n\r");

                        //zigbee API
                        strcpy(Message,szReturnStr);
                        len = strlen(Message);
                        Xbee_Frame(szReturnStr,Message,len,Coord_Addr);*/
                        
                        break;
		case COMMAND_NONE:
		default:
			// Unknown Command (not found in enum)
			RuntimeError(RX_MSG_ERROR_UNKNOWN_COMMAND);
			strcpy(szReturnStr, " ERR\n\r");
			return  FALSE;

	}


	return TRUE;

}



// this case-insensitive version of strncmp() is here because Microchip does not supply one

char strnicmp(const char* pStr1, const char *pStr2, size_t Count)
{
    char c1, c2;
    char  v;

    do {
        c1 = *pStr1++;
        c2 = *pStr2++;
        /* The casts are necessary when pStr1 is shorter & char is signed */
        v = (UINT) tolower(c1) - (UINT) tolower(c2);

    } while ((v IS 0) AND (c1 IS_NOT SZ_TERM) AND (c2 IS_NOT SZ_TERM) AND (--Count > 0) );

    return v;
}

void status()
{
    RTCC_DATE_TIME CurrentDateTime;
    PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
    LOCAL SmartTrakOrientation SPAOrientation;float el=0;
    int nStatus = 0;char temp[20];
         if (ReadRTCCDateTime(ptrDateTime) IS_NOT TRUE)
        {
                RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
                strcpy(szReturnStr, "0:0:0:0:0:0:0:0:0:0\n\r");
                strcpy(Message,szReturnStr);
                len = strlen(Message);
                Xbee_Frame(szReturnStr,Message,len,Coord_Addr);
                return FALSE;
        }
        else
        {
            IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr, ptrDateTime);
        }
        strcat(szReturnStr, ":");
        IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
        el=SPAOrientation.fElevation;
        ConvertSPAtoLocalOrientation(&SPAOrientation);

        strcat(szReturnStr,(const char *)ftoa2(SPAOrientation.fAzimuth, &nStatus));
        strcat(szReturnStr, ":");

        if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_TRACKING)
        {
            IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
            PanelPositionFSM(&SPAOrientation);
            
            strcat(szReturnStr,(const char *)ftoa(SPAOrientation.fAzimuth , &nStatus));
            strcat(szReturnStr, ":");
        }
        else
        {
            if((MAN_WEST == 1)OR(MAN_EAST == 1))
            {
            strcat(szReturnStr,(const char *)ftoa(pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
            strcat(szReturnStr, ":");
            }
            else
            {
            //strcat(szReturnStr,);
            strcat(szReturnStr, "0.0:");
            }
        }

        #ifdef USE_MMA8452Q_INCLINOMETER
           // strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
           // strcat(szReturnStr, ":");

            strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
            strcat(szReturnStr, ":");
        #endif

        //IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr + strlen(szReturnStr), ptrDateTime);
        //strcat(szReturnStr, ":");
        //4digits    x1x2x3x4
        //    x1- 0POWER_UP/1INIT/2TRACKING/3HOLD/4NIGHT_STO/5WIND_STOW/6BT
        //    x2- 1MAN/2AUTO     x3- 0stopped/1West/2East
        //    x4- 0MotorStooped/1Motor running
        BYTEtoASCIIstr(SPS, temp);
        
            if(((int)SPS < 1) || ((int)SPS > 6))
            strcat(szReturnStr,"07");
            else
             strcat(szReturnStr, temp);
        
        if(ptrRAM_SystemParameters->ucTracking_Mode IS MODE_MANUAL)
        {
            if(MAN_EAST == 1)
                strcat(szReturnStr, "12");  //east
            else if(MAN_WEST == 1)
                strcat(szReturnStr, "11");  //West
            else if(MAN_STOW == 1)
                strcat(szReturnStr, "13");  //stow
            else
                strcat(szReturnStr, "10");
        }
        else
        {
            if(bTrackerDirection == 1)
                strcat(szReturnStr, "21");  //West
            else if(bTrackerDirection == 2)
                strcat(szReturnStr, "22");  //east
            else
                strcat(szReturnStr, "20");  //stop
        }

        if (IsCommandComplete(AXIS_AZIMUTH) IS_TRUE)
            strcat(szReturnStr, "0");
        else
            strcat(szReturnStr, "1");
        #ifdef add_ext
        //adding lat and long attributes
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLatitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLongitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAltitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fTimeZone, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2((float)ptrRAM_SystemParameters->bBacktrackingEnabled, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSunShadowStartAngleDegrees, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Forward, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Forward, &nStatus));
        strcat(szReturnStr, ":");
        INT32UtoASCIIstr((BUILD_YEAR*10000+BUILD_MONTH*100+BUILD_DAY),INT32U_WIDTH,temp);
        strcat(szReturnStr,temp);
        strcat(szReturnStr, ":");
        BYTEtoASCIIstr(ptrRAM_SystemParameters->fSingle_stop_days, temp);
        strcat(szReturnStr,temp);
         #endif
        strcat((char *)szReturnStr, ":");
        strcat((char *)szReturnStr, (char *)ftoa2(BMS_V, &nStatus));
        strcat((char *)szReturnStr, ":");
        strcat((char *)szReturnStr, (char *)ftoa2(M1C, &nStatus));
        strcat(szReturnStr, ":");
        strcat((char *)szReturnStr, (char *)ftoa2(BTemp, &nStatus));

        strcat(szReturnStr, "\n\r");
        //zigbee API
        strcpy(Message,szReturnStr);
        len = strlen(Message);
        Xbee_Frame((char *)szReturnStr,Message,len,Coord_Addr);
}

void attributes(void)
{
    int nStatus = 0;
    // strcpy(szReturnStr,IdentityStr);
        //strcat(szReturnStr, ":");
        strcpy(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLatitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fLongitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAltitude, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fTimeZone, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2((float)ptrRAM_SystemParameters->bBacktrackingEnabled, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSunShadowStartAngleDegrees, &nStatus));
        strcat(szReturnStr, ":");
//       strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward, &nStatus));
//        strcat(szReturnStr, ":");
//        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse, &nStatus));
//        strcat(szReturnStr, ":");
//        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Forward, &nStatus));
//        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse, &nStatus));
        strcat(szReturnStr, ":");
        strcat(szReturnStr,(const char *)ftoa2(ptrRAM_SystemParameters->fSingle_SoftLimit_Forward, &nStatus));
        strcat(szReturnStr, "\n\r");
        strcpy(Message,szReturnStr);
        len = strlen(Message);
        Xbee_Frame(szReturnStr,Message,len,Coord_Addr);
}



// end of RxMessage.c

