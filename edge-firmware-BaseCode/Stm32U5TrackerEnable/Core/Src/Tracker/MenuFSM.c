// *************************************************************************************************
//										M e n u F S M . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Debug Serial Menu
//
// *************************************************************************************************

// NOTE: there are no divide-by-0 checks in this file, because it is only used under manual control
#include "main.h"
#include "GenericTypeDefs.h"	// includes stddef.h

#include "config.h"				// compile time configuration definitions


#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "HardwareProfile.h"

#include "EventFlags.h"			// event flag definitions and globals
#include "SerialPort.h"
#include "SerialDisplay.h"		// display functions for menus
#include "MenuFSM.h"
#include "StrConversions.h"		// ASCII string <==> numeric conversions
#include "ftoa.h"
#include "DS3232.h"
#include "UpdateParameters.h"
#include "RTCC.h"				// RTCC formatting
#ifdef USE_PCA9554_IO			// enable in config.h ONLY if PCA9554 hardware is present
#include "PCA9554.h"		// SPI expander definitions
#endif	// USE_PCA9554_IO
#ifdef USE_MMA8452Q_INCLINOMETER
#include "mma845x.h"         // MMA845xQ macros
#include "Inclinometer.h"
#endif	//  USE_MMA8452Q_INCLINOMETER


//#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
//#include "ADCRead.h"			// adc access functions
#include "Debounce.h"			// Input Switch debounce functions
#include "SunPosition.h"		// Sun Position Calculations
#include "PanelPositionFSM.h"

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
#include "MotionSensor.h"		// Motion (Hall) Sensor functions
#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations
#include "MotionStats.h"		// motion statistics for reporting
#include "MoveSequenceFSM.h"	// move sequence FSM

#include "ButtonProcessingFSM.h"// Button and user input processing


#include "AppTimer.h"			// for RS-232 timeouts, not currently implemented
#include "CoordTranslate.h"		// coordinate translation and formatting
#include "BMS.h"
#include <ctype.h>				// tolower()
#include <string.h>				// Microchip string functions

#include "PeripheralCallbacks.h"
#include "lis2hh12_reg.h"
#ifdef DEFINE_GLOBALS
#error "DEFINE_GLOBALS not expected here"
#endif



//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------
extern BOOL gIsMma845Enabled;
enum tagMenuFSMErrors
{
	MENU_FSM_ERROR_NONE = MENU_FSM_ERROR_BASE,
	MENU_FSM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	MENU_FSM_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	MENU_FSM_ERROR_INVALID_STATE,			// 3 not a valid state
	MENU_FSM_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	MENU_FSM_ERROR_INVALID_SELECTION,		// 5 not a valid selection
	MENU_FSM_ERROR_UNKNOWN_COMMAND,			// 6 not a valid command
	MENU_FSM_ERROR_INVALID_PARAMETER,		// 7 not a valid parameter selection
	MENU_FSM_ERROR_OUT_OF_RANGE_PARAMETER,	// 8 not a valid parameter selection
	MENU_FSM_ERROR_INVALID_MENU,			// 9 not a valid state
	MENU_FSM_ERROR_BUFFER_OVERFLOW,			// A menu string too long
	MENU_FSM_ERROR_COMMAND_OVERRUN,			// B command not executed or complete overrun

	MENU_FSM_UNPROCESSED_EVENT = MENU_FSM_ERROR_BASE + 0x0F

};


//-------------------------------------------------------------------------------------------------------
// Forward References - File Local function Definitions
//-------------------------------------------------------------------------------------------------------
// menu functions
BOOL DisplayMenu(void);
BOOL DisplayStatus(void);
BOOL DisplayResult(void);

int MainMenu(char);
int UtilityMenu(char);
int StatusMenu(char);
int MotionSettingsMenu( char);
int OpenLoopMoveMenu(char);
int ClosedLoopMoveMenu( char);
int MoveSequenceMenu( char cKeystroke);
int SystemParametersMenu( char);
int LocationParametersMenu( char);
int AzimuthParametersMenu(char);
int ElevationParametersMenu(char);
int BacktrackingParametersMenu(char);
int SetRTCCDateTimeMenu(char);
int ReadInputsMenu(char);
int TestMenu(char);
int UpdatedMainMenu(char cKeystroke);

void Orientation_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);	  // general purpose
void SPA_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);				// general purpose
void SPA_Backtrack_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation);

void CurrentMechanicalOrientation_Format(char *ptrOutputStr);
void CurrentLocalOrientation_Format(char *ptrOutputStr);							// accounts for offsets
//unsigned char xb_rx[20],m=0;

//unsigned char xbee_pkt[1000],Coor_Addr[8];
//unsigned char ar[20],i=0;
char dispstr[60];

//-------------------------------------------------------------------------------------------------------
// File Local - Global Variables
//-------------------------------------------------------------------------------------------------------

FILE_GLOBAL_INIT enum tagMotors eClosedLoopActiveMotor = MOTOR_NONE;	// keep track of currently active motor, as determined by ClosedLoop menu selection

// indicies for finite-length streaming data lists

FILE_GLOBAL_INIT WORD	fgwErrorDisplayIndex = 0;
FILE_GLOBAL_INIT WORD	fgwMotionErrorDisplayIndex = 0;		// was MIN_SPEED_INDEX;
FILE_GLOBAL_INIT BYTE	fgbPolarAxisMoveDisplayIndex = 0;
FILE_GLOBAL_INIT BYTE	fgbStreamSunAnglesState = 0;		// breaks streaming Sun Angles into substates
float      BMS_V,M1C,M2C,BTemp,OP1,OP2;
//-----------------------------------------------
// Keep track of Menu selection
PRIVATE_INIT BYTE bMenuFSMSequenceCtr = 0;					// substate sequence counter
PRIVATE_INIT BYTE fSubStateStatus = SUBSTATE_NOT_DONE;		// substate status flag

#define	NOTHING_SELECTED			0
#define MENU_SELECTED				1
#define COMMAND_SELECTED			2
#define PARAMETER_SELECTED			3
#define DATA_STREAM_SELECTED		4
#define REALTIME_DATA_SELECTED		5


PRIVATE_INIT int fSelection = MENU_SELECTED;


//-----------------------------------------------
// Menu FSM states
enum tagMenuStates
{
	ST_MENU_INIT,						// initial state, only at power up

	ST_MENU_DISPLAY_MENU,
	ST_MENU_WAIT_FOR_KEYSTROKE,			// waiting for menu (or command) selection keystroke
	ST_MENU_WAIT_FOR_PARAMETER_AND_CR,	// collecting keystrokes and waiting for CR terminator
	//	ST_MENU_WAIT_FOR_CR,
	ST_MENU_PROCESS_COMMAND,
	ST_MENU_DISPLAY_STREAMING_DATA,		// streaming data is generated by MenuFSM(), and may appear as under-sampled data (i.e not all data is displayed)
	ST_MENU_CHECK_FOR_TERMINATING_KEYSTROKE,
	ST_MENU_DISPLAY_REALTIME_DATA,		// realtime streaming data is generated elsewhere, and is only controlled (on/off) by MenuFSM
	ST_MENU_DISPLAY_RESULT

};

PRIVATE_INIT enum tagMenuStates stMenuState = ST_MENU_INIT;


//-----------------------------------------------
// keep track of selected menu
enum tagMenus
{
	MAIN_MENU,
	UTILITY_MENU,
	STATUS_MENU,
	MOTION_SETTINGS_MENU,
	OPEN_LOOP_MOVE_MENU,
	CLOSED_LOOP_MOVE_MENU,
	MOVE_SEQUENCE_MENU,
	SYSTEM_PARAMETERS_MENU,
	LOCATION_PARAMETERS_MENU,
	AZIMUTH_PARAMETERS_MENU,
	ELEVATION_PARAMETERS_MENU,
	BACKTRACKING_PARAMETERS_MENU,
	RTCC_DATE_TIME_MENU,
	READ_INPUTS_MENU,
	TEST_MENU
};

static enum tagMenus eCurrentMenu = MAIN_MENU;
extern unsigned int	pgwRuntimeErrorBuffer[ERROR_BUFFER_SIZE];

//-----------------------------------------------
// substate to keep track of multi-line menu display
// only one line displayed per call to MenuFSM()
enum tagMenuLine
{
	MENU_SINGLE_LINE,				// menu display has just one line (not actually possible)
	MENU_LINE,						// line 1 to n
	MENU_LAST_LINE					// last line to display
};


PRIVATE_INIT enum tagMenuLine eMenuLine = MENU_LINE;


//-----------------------------------------------
// substate to keep track of multi-line result display
enum tagResultLine
{
	RESULT_SINGLE_LINE,				// result display has just one line
	RESULT_LINE,					// line 1 to n
	RESULT_LAST_LINE				// last line to display
};


PRIVATE_INIT enum tagResultLine eResultLine = RESULT_LINE;
PRIVATE WORD fgwDisplayLineCtr;

// specify which multi-line output block to display
// the multi-line blocks are display 1 line per pass through MenuFSM()
enum tagResultDisplay
{
	RESULT_NONE,
	RESULT_STATUS_LAST_AZ_MOVEMENT,
	RESULT_STATUS_LAST_EL_MOVEMENT,
	RESULT_READ_REGISTERS_MTN_SENSOR_SETTINGS,
	RESULT_READ_REGISTERS_PWM_SETTINGS,
	RESULT_MOTOR_PWM_WIDTHS,
	RESULT_READ_INPUTS_MOTION_SENSORS,
	RESULT_READ_REGISTERS_TIMERS,
	RESULT_READ_REGISTERS_ADC_SETTINGS,
	RESULT_READ_INPUTS_ADC,
	RESULT_READ_INPUTS_SWITCHES,
	RESULT_READ_REGISTERS_RTCC,
	RESULT_READ_RTCC_RAM,
	RESULT_READ_REGISTERS_INCLINOMETER,
	RESULT_READ_INCLINOMETER_DATA,
	RESULT_READ_SUN_ANGLES,
	RESULT_READ_LOCATION_SETTINGS,
	RESULT_READ_BUILD_DATE

};

PRIVATE_INIT enum tagResultDisplay eResultType = RESULT_NONE;

//-----------------------------------------------
// specify status type to display
// streaming status should be limited to 1 line per pass through MenuFSM()
// keep track of selected streaming data type
enum tagStatus
{
	AZ_FSM_STATUS,						// continuous until a serial terminal key hit
	EL_FSM_STATUS,						// continuous until a serial terminal key hit
#ifdef MOTION_ERROR_TABLE
	MOTION_ERROR_STATUS,			// table of motion error values, streams until out of data OR serial terminal key hit
#endif
	MOTOR_STATUS,						// continuous until a serial terminal key hit
	INPUT_SWITCH_STATUS,				// continuous until a serial terminal key hit
#ifdef USE_MMA8452Q_INCLINOMETER
	INCLINOMETER_STATUS,			// continuous until a serial terminal key hit
	INCLINOMETER_AVERAGE_STATUS,	// continuous until a serial terminal key hit
#endif
#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	POLAR_AXIS_MOVE_STATUS,
#endif
	SUN_ANGLES_STATUS,
	BMS_STATUS,
	RUNTIME_ERROR_STATUS				// table of runtime error values, streams until out of data OR serial terminal key hit
};

PRIVATE_INIT enum tagStatus eCurrentStatus = AZ_FSM_STATUS;


//-----------------------------------------------
// substate to keep track of status display
enum tagStreamLine
{
	STREAM_FIRST_LINE,
	STREAM_LINE,
	STREAM_LAST_LINE
};

PRIVATE_INIT enum tagStreamLine eStreamLine = STREAM_FIRST_LINE;

//-----------------------------------------------
// keep track of parameter to update
// see UpdateParameters.h for tagParameter enum, function prototype

PRIVATE_INIT enum tagParameter eParameter = PARAMETER_NONE;

// parameter type is needed for input validation and proper conversions back and forth
// placed here, because these are ONLY used by MenuFSM.c
enum tagParameterType
{
	PARAMETER_TYPE_NONE,
	PARAMETER_TYPE_DECIMAL,
	PARAMETER_TYPE_BCD,
	PARAMETER_TYPE_HEX,
	PARAMETER_TYPE_FLOATING_PT
};

PRIVATE_INIT enum tagParameterType eParameterType = PARAMETER_TYPE_NONE;


//-----------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Menu Display Functions, including fixed Menu Strings
//-------------------------------------------------------------------------------------------------------

// starts with:
//		Clear screen: ESC [ 2 J
//		Home cursor: ESC [ ROW ; COL H 

// note this is a pointer to an array of pointers
const char **fgpMenuText;

//lint -e786  error 786: (Info -- String concatenation within initializer)
// this is setup as an array of pointers
FILE_GLOBAL ARRAY  const char *pstrMainMenuText[] =
{
		//	"\x1B[2J\x1B[0;0H==SmartTrak Solar Panel Controller: (" __DATE__ "," __TIME__ ")",
		"\n\r==SmartTrak Solar Panel Controller: (" __DATE__ "," __TIME__ "):Version-"Version_SAT"",
#ifdef USE_ELEVATION
		"1) * Run UP (fwd) to end",
#endif
		"2) * Run EAST (rev) to end ",
		"3) * Run WEST (fwd) to end",
#ifdef USE_ELEVATION
		"4) * Run DOWN (rev) to end",
#endif
		"   s) * Stop",
		"----------------------------",
		"5) * Calibrate (Find End Points)",
		"6) Current Orientation = Sun Position (Force PV)",
		"----------------------------",
		"7) Enter Azimuth Offset",
#ifdef USE_ELEVATION
		"8) Enter Elevation Offset",
#endif
		"9) Display Current Orientation",
		"a) Display Current SPA Values",
		"b) Clear Current Orientation",
		"c) Clear RTCC NV RAM",
		"d) * Service (Manual/Auto Start)",
		"e) Write Parameters to Flash",
		"----------------------------",
		"f) stream Runtime Errors",
		"r) * Reset",
		"----------------------------",
		"g) Utility Menus",
		"h) Remote Mode",
		"i) Activate Internal Accelerometer",
		"j) Disable Internal Accelerometer",
		//	"end",
		""
};

// this is setup as an array of pointers
FILE_GLOBAL ARRAY  const char *pstrUtilityMenuText[] =
{
		"\n\r",
		"1) Status",
		"2) Motion Settings",
		"----------------------------",
		"3) Open Loop Move",
		"4) Closed Loop Move",
		"5) Move Sequence",
		"----------------------------",
		"6) Set System Parameters",
		"7) Read Inputs/Set Outputs",
		"8) Test Menu",
		" x) Exit Menu",
		//	"end",
		""
};


// ==>> need to add 'stream hall sensor'
FILE_GLOBAL ARRAY  const char *pstrStatusMenuText[] =
{
		"\n\r",
#ifdef USE_ELEVATION
		"\t1) last Elevation move",
#endif
		"\t2) last Azimuth move",
		"\t3) last Motion Speed Errors",
		"\t------------------------------",
#ifdef USE_ELEVATION
		"\t4) stream Elevation FSM States",
#endif
		"\t5) stream Azimuth FSM States",
		"\t6) stream Motion Status",
		"\t7) stream Input Switches",
		"----------------------------",
		"\t8) stream Inclinometer Values",
		"\t9) stream Averaged Inclinometer Values",
		"\ta) stream Single Polar Axis SPA Tracking Moves",
		"\tb) stream Sun Angles",
		"\tc) stream BMS"
		"----------------------------",
		"\te) stream Runtime Errors",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrMotionSettingsMenuText[] =
{
		"\n\r",
		"\t1) Motion Sensor (Input Capture) register values",
		"\t2) PWM (Output Capture) register values",
#ifdef USE_PWM_DUTY_CYCLE
		"\t3) current PWM duty cycles",
		"\t4) change Elevation PWM duty cycle",
		"\t5) change Azimuth PWM duty cycle",
#endif
		"\t x) Exit Menu",
		//	"end",
		""
};

FILE_GLOBAL ARRAY const char *pstrOpenLoopMoveMenuText[] =
{
		"\n\r",
		"\t Open Loop Menu does NOT use Motion FSMs",
#ifdef USE_ELEVATION
#ifdef USE_ELEVATION_ON_OFF_LINEAR_DRIVE
		"\t Elevation ON/OFF Drive",
#else
		"\t1) change Elevation PWM duty cycle",
#endif
		"\t2) run up (fwd)",
		"\t3) run down (rev)",
		"\t------------------------------",
#endif	//  USE_ELEVATION
#ifdef USE_AZIMUTH
#ifdef USE_PWM_DUTY_CYCLE
		"\t4) change Azimuth PWM duty cycle",
#endif
		"\t5) run right (fwd)",
		"\t6) run left(rev)",
		"\t------------------------------",
#endif
		"\ts) stop (open loop ONLY)",
		"\tr) reset (NOT in Debug)",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY const char *pstrClosedLoopMoveMenuText[] =
{
		"\n\r",
		"\t Closed Loop Menu uses MotionPhaseFSM and MotionFSM",
		"\t1) set move distance (degrees)",
#ifdef USE_ELEVATION
		"\t2) run (degrees) up",
		"\t3) run (degrees) down (rev)",
#endif	//  USE_ELEVATION
#ifdef USE_AZIMUTH
		"\t4) run (degrees) right",
		"\t5) run (degrees) left (rev)",
#endif	// USE_AZIMUTH
		"\t----------------------------",
#ifdef USE_ELEVATION
		"\t6) slew up (fwd) to end",
		"\t7) slew down (rev) to end ",
#endif	//  USE_ELEVATION
#ifdef USE_AZIMUTH
		"\t8) slew right (fwd) to end",
		"\t9) slew left (rev) to end",
#endif	// USE_AZIMUTH
		"\t----------------------------",
#ifdef USE_ELEVATION
		"\ta) run one (motor gearbox) rotation up (fwd 5 degrees)",
		"\tb) run one (motor gearbox) rotation down (rev 5 degrees)",
#endif	//  USE_ELEVATION
#ifdef USE_AZIMUTH
		"\tc) run one (motor gearbox) rotation right (fwd 5 degrees)",
		"\td) run one (motor gearbox) rotation left (rev 5 degrees))",
#endif	// USE_AZIMUTH
		"\ts) stop",
		"\tr) reset (NOT in Debug)",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrMoveSequenceMenuText[] =
{
		"\n\r",
		"\t MoveSequence Menu uses MoveSequenceFSM, MotionPhaseFSM and MotionFSM",
		"\t1) find end points",
		"\t2) move to (degrees, degrees) position",
		"\t3) move to center position",
		"\t4) move to Night Stow",
		"\t5) move to Wind Stow",
		"\t--------------------------",
		"\t7) SPA Calculation (no movement)",
		"\t8) SPA Track (Calculated Movement)",
		"\t--------------------------",
		"\t6) set Step size (degrees, default 0.0 = no move)",
#ifdef USE_ELEVATION
		"\t9) move 1 Step up (fwd)",
		"\ta) move 1 Step down (rev)",
#endif	//  USE_ELEVATION
#ifdef USE_AZIMUTH
		"\tb) move 1 Step right (fwd)",
		"\tc) move 1 Step left (rev)",
		"\td) incremental move 1 Step right (fwd)",
		"\te) incremental move 1 Step left (rev)",
#endif	// USE_AZIMUTH
		"\t--------------------------",
		"\ts) stop",
		"\tr) reset (NOT in Debug)",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrSystemParametersMenuText[] =
{
		"\n\r",
		"\t1) Unit Location",
		"\t2) Azimuth Settings",
		"\t3) Elevation Settings",
		"\t4) SingleAxis Settings",
		"\t5) Write above Parameters to SPI Flash",
		"\t--------------------------",
		"\t6) Initialize SPI Flash",
		"\t7) Set RTCC Date/Time",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrUnitLocationParametersMenuText[] =
{
		"\n\r Unit Location Parameters, changes RAM values only",
		"\t\t1) fLatitude",				// R/W, In degrees. Range = +90 degrees to -90 degrees, Positive for north of the equator.
		"\t\t2) fLongitude",			// R/W, In degrees. Range = -180 to +180deg, Positive for east of GMT
		"\t\t3) fAltitude (m)",			// Float R/W In meters. Above sea level.
		"\t\t4) fRefraction",			// Float R/W No units. Value is around 1.
		"\t\t5) fTimeZone (GMT)",		// Float R/W Offset from GMT (like 5.5 for India)
		"\t\t6) ucTracking_Mode",		// Int (enum) R/W Values: Tracking, Night Stow, Wind Stow
		// TBD: make it read only? May need write permission for testing purpose
		"\t x) Exit Menu",
		//			"end",
		""
};


FILE_GLOBAL ARRAY  const char *pstrAzimuthParametersMenuText[] =
{
		"\n\r Azimuth Parameters, changes RAM values only",
		"\t\t1) fAZ_Offset",				// Float R/W center offset from 0 (due south) May be set from any orientation. Units: degrees
		"\t\t2) fAZ_SoftLimit_Reverse",	// Float R/W Azimuth soft limit REVERSE (left/down). Units: degrees
		"\t\t3) fAZ_SoftLimit_Forward",	// Float R/W Azimuth soft limit FORWARD (right/up). Units: degrees
		"\t\t4) fAZ_DeadBand",			// Float R/W TBD Units: degrees
		"\t\t5) fAZ_NightStowThreshold",	// Float R/W Default = 0 Units: degrees
		"\t\t6) fAZ_NightStowPosition",	// Float R/W Default = 0 (remain in place) Units: degrees
		"\t\t7) fAZ_WindStowPosition",	// Float R/W Default = 0 (remain in place) Units: degrees

		//"\t8) fAZ_PVpos",				// Float R only This is the present position of the PV panel. This is calculated based on AZ_PulseCount.
		//"\t9) unAZ_PulseCount",		// Int R only This is the number of hall pulses.
		//"\ta) bAZ_En_AutoCal",		// Bool R/W
		//"\tb) bAZ_AutoMove",			// Bool, R/W	True: Move the motor to AZ_SetPoint
		//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
		//"\tc) fAZ_cal_param1",		// Float R/W TBD: could be needed for cal correction
		//"\td) fAZ_cal_param2",		// Float R/W TBD: could be needed for cal correction
		"\t\t x) Exit Menu",
		""
};

FILE_GLOBAL ARRAY  const char *pstrElevationParametersMenuText[] =
{
		"\n\r Elevation Parameters, changes RAM values only",
		"\t\t1) fEL_Offset",				// Float R/W center offset from 45 degrees (45 degrees above horizon May be set from any orientation. Units: degrees
		"\t\t2) fEL_SoftLimit_Reverse",	// Float R/W Elevation REVERSE (left/down). Units: degrees
		"\t\t3) fEL_SoftLimit_Forward",	// Float R/W Elevation FORWARD soft limit maximum. Units: degrees
		"\t\t4) fEL_DeadBand",			// Float R/W TBD
		"\t\t5) fEL_NightStowThreshold",	// Float R/W Default = 0 Units: degrees
		"\t\t6) fEL_NightStowPosition",	// Float R/W Default = 0 (remain in place)
		"\t\t7) fEL_WindStowPosition",	// Float R/W Default = 0 (remain in place)

		//"\t7) fEL_SetPoint",			// Float R/W This comes from SPA_calc, but limited by soft limits
		//"\t8) fEL_PVpos",				// Float R only This is the present position of the PV panel. This is calculated based on EL_PulseCount.
		//"\t9) unEL_PulseCount",		// Int R only This is the number of hall pulses.
		//"\ta) bEL_En_AutoCal",		// Bool R/W
		//"\tb) bEL_AutoMove",			// Bool, R/W	True: Move the motor to EL_SetPoint
		//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
		//"\tc) fEL_cal_param1",		// Float R/W TBD: could be needed for cal correction
		//"\td) fEL_cal_param2",		// Float R/W TBD: could be needed for cal correction
		"\t\t x) Exit Menu",
		//			"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrBacktrackingParametersMenuText[] =
{
		"\n\r Backtracking Parameters",
		"\t\t1) Backtracking Enabled (0 = Disabled, 1 = Enabled)",
		"\t\t2) Panel shadow start angle (degrees)",
		"\t\t3) Sun shadow start angle (degrees)",
		"\t\t4) Sun shadow start Height (units?)",
		"\t\t5) SingleAxis Reverse Limit",
		"\t\t6) SingleAxis Forward Limit",
		"\t\t7) SingleAxis start date",
		"\t\t8) SingleAxis stop days",
		"\t\t x) Exit Menu",
		//			"end",
		""
};


FILE_GLOBAL ARRAY  const char *pstrSetRTCCDateTimeMenuText[] =
{
		"\t\n\r",
		"\t\t1) seconds",
		"\t\t2) minutes",
		"\t\t3) hours",
		"\t\t4) day (1-7, 1 = Mon)",
		"\t\t5) date",
		"\t\t6) month",
		"\t\t7) year 20xx",
		"\t\t x) Exit Menu",
		//			"end",
		""
};


FILE_GLOBAL ARRAY  const char *pstrReadInputsMenuText[] =
{
		"\n\r",
		"\t1) Motion (Hall) Sensor Inputs",
		"\t2) Motion (Hall) Sensor counter values",
		"\t--------------------------",
		"\t3) Timer values",
		"\t4) ADC register values",
		"\t5) ADC Measurement values",
		"\t6) Switch Input values",
		"\t--------------------------",
		"\t7) Read RTCC register values",
		"\t8) Read RTCC RAM",
#ifdef USE_INCLINOMETER_FEEDBACK
		"\t--------------------------",
		"\t9) Read Inclinometer register values",
		"\ta) Read Inclinometer Data",
		"\t--------------------------",
#endif
		"\tb) Set LED State",
		"\tc) Location Values",
		"\td) Compile Date",
		"\t x) Exit Menu",
		//		"end",
		""
};

FILE_GLOBAL ARRAY  const char *pstrTestMenuText[] =
{
		"\n\r",
		"\t1) Watchdog Timer",
		"\t2) Exception Handler",
		"\t x) Exit Menu",
		//		"end",
		""
};

// *****************************************************************************
//				M e n u F S M ( ) 
// *****************************************************************************

// this is the TOP level menu function, called every 25mS from the sequencer
unsigned char inputbuffer[100],buf[10],c;
//stMenuState = ST_MENU_INIT;
void MenuFSM(void)
{
	//   static unsigned char f,buff[100];
	static UINT16 bInputBufferIndex = 0;
	BOOL bValueValid;
	char cKey;

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
	//	uart_send("\r\nState Menu:");
	//	INT32UtoASCIIstr(stMenuState,sizeof(buf),(char *)buf);
	//	uart_send((char *)buf);
	//	uart_send("\r\n");
	switch(stMenuState)
	{
	case ST_MENU_INIT:				// initial state, only at power up
		//				uart_send("\r\nMenu Init.\r\n");
		stMenuState = ST_MENU_DISPLAY_MENU;
		fSubStateStatus = SUBSTATE_NOT_DONE;
		bMenuFSMSequenceCtr = 0;
		eCurrentMenu = MAIN_MENU;
		eSerialOutputMode = SER_MODE_MENU;
//		uart_send("\r\nMenu Init end.\r\n");
		break;

	case ST_MENU_DISPLAY_MENU:
		// check for menu display complete
//		uart_send("\r\n");
		if ((fSubStateStatus == SUBSTATE_DONE) && (eMenuLine == MENU_LINE))
		{
			// we are done with displaying ONE line, but we are not done with ALL lines
			// restart state AFTER initialization substate, so we do not restart display line counter
			fSubStateStatus = SUBSTATE_NOT_DONE;
			bMenuFSMSequenceCtr = 1;
		}
		// check for done with all available data
		else if ((fSubStateStatus == SUBSTATE_DONE) && (eMenuLine == MENU_LAST_LINE))
		{
			// we are done with last line and sequence is complete, so bump state
			fSubStateStatus = SUBSTATE_NOT_DONE;
			bMenuFSMSequenceCtr = 0;
			stMenuState = ST_MENU_WAIT_FOR_KEYSTROKE;
		}
		// else display is not complete, so no state change
		break;

	case ST_MENU_WAIT_FOR_KEYSTROKE:
		// check for keystroke received
		if (fSubStateStatus == SUBSTATE_DONE)
		{
			// sequence is complete, so bump state
			fSubStateStatus = SUBSTATE_NOT_DONE;
			bMenuFSMSequenceCtr = 0;
			stMenuState = ST_MENU_PROCESS_COMMAND;
		}
		break;

		//		case ST_MENU_WAIT_FOR_CR:
		//			break;

	case ST_MENU_PROCESS_COMMAND:
		// check for result
		switch(fSelection)
		{
		case MENU_SELECTED:
			stMenuState = ST_MENU_DISPLAY_MENU;
			break;

		case COMMAND_SELECTED:
			stMenuState = ST_MENU_DISPLAY_RESULT;
			fgwDisplayLineCtr = 0;
			break;

		case PARAMETER_SELECTED:
			stMenuState = ST_MENU_WAIT_FOR_PARAMETER_AND_CR;		// ST_MENU_WAIT_FOR_CR;
			break;

		case DATA_STREAM_SELECTED:
			stMenuState = ST_MENU_DISPLAY_STREAMING_DATA;
			break;

		default:
			RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
			stMenuState = ST_MENU_DISPLAY_MENU;
			break;
		}

		// clear flags
		fSubStateStatus = SUBSTATE_NOT_DONE;
		bMenuFSMSequenceCtr = 0;
		break;

		case ST_MENU_WAIT_FOR_PARAMETER_AND_CR:
			// collecting keystrokes and waiting for CR terminator
			// receieved keystroke(s) are processed in last substate, below
			if (fSubStateStatus == SUBSTATE_DONE)
			{
				// sequence is complete, so bump state
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;

				// stMenuState = ST_MENU_PROCESS_COMMAND;		// would it make more sense to move parameter processing to a separate state?
				stMenuState = ST_MENU_DISPLAY_MENU;
			}
			break;


		case ST_MENU_DISPLAY_STREAMING_DATA:
			// check for first entry into state
			if ((fSubStateStatus == SUBSTATE_NOT_DONE) && (bMenuFSMSequenceCtr == 0))
			{
				eStreamLine = STREAM_FIRST_LINE;				// this is not actually used anywhere..
			}
			// check for display of ONE LINE of streaming data complete
			else if ((fSubStateStatus == SUBSTATE_DONE) && (eStreamLine == STREAM_LINE))
			{
				// single line sequence is complete, so bump state to check for terminating keystroke
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
				stMenuState = ST_MENU_CHECK_FOR_TERMINATING_KEYSTROKE;;
			}
			// check for done with all available data
			else if ((fSubStateStatus == SUBSTATE_DONE) && (eStreamLine == STREAM_LAST_LINE))
			{
				// sequence is complete, so bump state
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
				stMenuState = ST_MENU_DISPLAY_MENU;
			}

			// display is not complete, so no state change
			break;

		case ST_MENU_CHECK_FOR_TERMINATING_KEYSTROKE:
			// check for keystroke received
			if (fSubStateStatus == SUBSTATE_DONE)
			{
				switch(fSelection)
				{
				case NOTHING_SELECTED:
					// no keystroke received, so display another line of streaming data
					stMenuState = ST_MENU_DISPLAY_STREAMING_DATA;
					break;

				case MENU_SELECTED:
					// sequence is complete (keystroke received), so bump state back to re-display menu
					stMenuState = ST_MENU_DISPLAY_MENU;
					break;

				default:
					RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
					stMenuState = ST_MENU_DISPLAY_MENU;
					break;
				}

				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
			}
			// else not done with substates
			break;

		case ST_MENU_DISPLAY_REALTIME_DATA:
			// check for keystroke received
			if ((fSubStateStatus == SUBSTATE_DONE) && (fSelection == MENU_SELECTED))
			{
				// sequence is complete (keystroke received), so bump state back to re-display menu
				stMenuState = ST_MENU_DISPLAY_MENU;

				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
			}
			// else not done with substates
			break;

		case ST_MENU_DISPLAY_RESULT:
			// check for display of ONE LINE of result complete
			if ((fSubStateStatus == SUBSTATE_DONE) && (eResultLine == RESULT_LINE))
			{
				// we are done with displaying ONE line, but we are not done with ALL lines
				// restart state (but do not restart display line counter)
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
			}

			// check for done with all available data
			else if ((fSubStateStatus == SUBSTATE_DONE) && ((eResultLine == RESULT_LAST_LINE) || (eResultLine == RESULT_SINGLE_LINE)) )
			{
				// we are done with last line and sequence is complete, so bump state
				fSubStateStatus = SUBSTATE_NOT_DONE;
				bMenuFSMSequenceCtr = 0;
				stMenuState = ST_MENU_DISPLAY_MENU;
			}

			// else display is not complete, so no state change
			break;

		default:
			RuntimeError(MENU_FSM_ERROR_INVALID_STATE);
			break;

	}

	// *************************************************
	//		    Process the Current or New State
	// *************************************************
	// State handler
	//	called one or more times for a state, depending on state implementation
	//	state transitions do NOT occur here
	//	no fallthrough state transitions - use fFSM_Execute flag to force immediate re-execute for eventless state changes
	//	NOTE: the states transition handler above must make sure that states are not processed multiple times, if doing so is inappropriate

	switch(stMenuState)
	{
	case ST_MENU_INIT:				// we should NEVER be in this state
		RuntimeError(MENU_FSM_ERROR_INVALID_STATE);
		break;

	case ST_MENU_DISPLAY_MENU:
		switch(bMenuFSMSequenceCtr)
		{
		case 0:
			// substate for first entry into state, setup to start display of the current menu
			switch(eCurrentMenu)
			{
			case MAIN_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrMainMenuText;
				break;

			case UTILITY_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrUtilityMenuText;
				break;

			case STATUS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrStatusMenuText;
				break;

			case MOTION_SETTINGS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrMotionSettingsMenuText;
				break;

			case OPEN_LOOP_MOVE_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrOpenLoopMoveMenuText;
				break;

			case CLOSED_LOOP_MOVE_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrClosedLoopMoveMenuText;
				break;

			case MOVE_SEQUENCE_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrMoveSequenceMenuText;
				break;

			case SYSTEM_PARAMETERS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrSystemParametersMenuText;
				break;

			case LOCATION_PARAMETERS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrUnitLocationParametersMenuText;
				break;

			case AZIMUTH_PARAMETERS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrAzimuthParametersMenuText;
				break;

			case ELEVATION_PARAMETERS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrElevationParametersMenuText;
				break;

			case BACKTRACKING_PARAMETERS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrBacktrackingParametersMenuText;
				break;

			case RTCC_DATE_TIME_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrSetRTCCDateTimeMenuText;
				break;

			case READ_INPUTS_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrReadInputsMenuText;
				break;

			case TEST_MENU:
				// initialize pointer to menu text
				fgpMenuText = pstrTestMenuText;
				break;
			default:
				RuntimeError(MENU_FSM_ERROR_INVALID_MENU);
				break;
			}

			fgwDisplayLineCtr = 0;							// initialize menu line counter
			++bMenuFSMSequenceCtr;							// force next substate on next FSM entry
			break;

			case 1:
				// substate to display a single line of the menu
				//					uart_send("\r\nTry to display menu\r\n");
				if(DisplayMenu() IS_FALSE)				// returns FALSE if nothing MORE to display
				{
					//						uart_send("\r\nDisplay menu Failed\r\n");
					fSubStateStatus = SUBSTATE_DONE;			// force exit from state; no need for 2nd substate
					++bMenuFSMSequenceCtr;							// force next substate on next FSM entry
				}
				break;

			default:
				RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
				break;

		}
		break;

		case ST_MENU_WAIT_FOR_KEYSTROKE:
			switch(bMenuFSMSequenceCtr)
			{
			case 0:
				// check for available keystroke
				memset(inputbuffer,0,sizeof(inputbuffer));
				if(uart_recv(inputbuffer,1) == HAL_OK)
				{
					pgcInputBuffer[0]=inputbuffer[0];
					++bMenuFSMSequenceCtr;		// force next substate on next FSM entry
					fSubStateStatus = SUBSTATE_DONE;
				}

#ifdef DOES_NOT_WORK_CORRECTLY	// kludge that does not work correctly
				else if ((eCurrentMenu == MOVE_SEQUENCE_MENU) && (IsMoveSequenceComplete() IS_FALSE))
				{
					// Kludge added to force menu re-display after completion of Move Sequence
					// fake a keystroke; it will not be processed until the move sequence has completed
					pgcInputBuffer[0] = 'z';
					++bMenuFSMSequenceCtr;		// force next substate on next FSM entry
					fSubStateStatus = SUBSTATE_DONE;
				}
#endif
				break;

			default:
				RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
				break;
			}
			break;

			//		case ST_MENU_WAIT_FOR_CR:
			//			break;

			case ST_MENU_PROCESS_COMMAND:
				switch(eCurrentMenu)
				{
				case MAIN_MENU:								// always single pass display
//					cKey = pgcInputBuffer[0];
					fSelection = MainMenu(pgcInputBuffer[0]);
//					fSelection = UpdatedMainMenu(pgcInputBuffer[0]);
					break;

				case UTILITY_MENU:							// always single pass display
					fSelection = UtilityMenu(pgcInputBuffer[0]);
					break;

				case STATUS_MENU:							// may be single pass or multiple pass display
					fSelection = StatusMenu(pgcInputBuffer[0]);
					break;

				case MOTION_SETTINGS_MENU:					// may be single pass or multiple pass display
					fSelection = MotionSettingsMenu(pgcInputBuffer[0]);
					break;

				case OPEN_LOOP_MOVE_MENU:								// always single pass display
					fSelection = OpenLoopMoveMenu(pgcInputBuffer[0]);
					break;

				case CLOSED_LOOP_MOVE_MENU:								// always single pass display
					fSelection = ClosedLoopMoveMenu(pgcInputBuffer[0]);
					break;

				case MOVE_SEQUENCE_MENU:					// always single pass display
					fSelection = MoveSequenceMenu(pgcInputBuffer[0]);
					break;

				case SYSTEM_PARAMETERS_MENU:
					fSelection = SystemParametersMenu(pgcInputBuffer[0]);
					break;

				case LOCATION_PARAMETERS_MENU:
					fSelection = LocationParametersMenu(pgcInputBuffer[0]);
					break;

				case AZIMUTH_PARAMETERS_MENU:
					fSelection = AzimuthParametersMenu(pgcInputBuffer[0]);
					break;

				case ELEVATION_PARAMETERS_MENU:
					fSelection = ElevationParametersMenu(pgcInputBuffer[0]);
					break;

				case BACKTRACKING_PARAMETERS_MENU:
					fSelection = BacktrackingParametersMenu(pgcInputBuffer[0]);
					break;

				case RTCC_DATE_TIME_MENU:
					fSelection = SetRTCCDateTimeMenu(pgcInputBuffer[0]);
					break;

				case READ_INPUTS_MENU:						// may be single pass or multiple pass display
					fSelection = ReadInputsMenu(pgcInputBuffer[0]);
					break;

				case TEST_MENU:
					fSelection = TestMenu(pgcInputBuffer[0]);
					break;

				default:
					RuntimeError(MENU_FSM_ERROR_INVALID_MENU);
					break;
				}
				break;

				case ST_MENU_WAIT_FOR_PARAMETER_AND_CR:				// ST_MENU_WAIT_FOR_CR:
					// collecting keystrokes and waiting for CR terminator
					switch(bMenuFSMSequenceCtr)
					{
					case 0:
					{
						bInputBufferIndex=0;
						++bMenuFSMSequenceCtr;
					}
					case 1:
						if(uart_recv(inputbuffer,1) == HAL_OK)
						{
							pgcInputBuffer[bInputBufferIndex] = inputbuffer[0];
							if (isprint(pgcInputBuffer[bInputBufferIndex]) != ZERO)
							{

								DisplayCharacter(pgcInputBuffer[bInputBufferIndex], NO_WAIT_FOR_DISPLAY);		// echo character; we are assuming this will complete before additional message below
							}

							// check for end of input
							if (pgcInputBuffer[bInputBufferIndex] == ASCII_CR)
							{
								// input is complete
								pgcInputBuffer[bInputBufferIndex] = '\0';			// terminate input string

								// check for non-zero length of input string (zero length string is interpreted as... ZERO)
								if (bInputBufferIndex > 0)
								{
									++bMenuFSMSequenceCtr;								// non-zero length string, so this must be the terminator for an entry; force next substate on next FSM entry for processing of parameter
								}
								else
								{
									fSubStateStatus = SUBSTATE_DONE;					// no input string, so we are done with the ENTIRE state
								}
							}

							// Check keystroke for valid type; numeric data ONLY. Note processing sequence order; CR and ESC are non-numeric, too!

							else if (pgcInputBuffer[bInputBufferIndex] == ASCII_ESC)
							{
								// abort substate
								fSubStateStatus = SUBSTATE_DONE;					// we are done with the ENTIRE state
								//++bMenuFSMSequenceCtr;								// force next substate on next FSM entry  <sek> 26 Feb 13
								break;
							}
							// Check keystroke for valid type according to parameter type; Note above sequence order: CR is non-numeric, too!
							else if ((eParameterType == PARAMETER_TYPE_DECIMAL) && (isdigit(pgcInputBuffer[bInputBufferIndex]) == ZERO))
							{
								// allow digits. ==> do we need to allow minus sign?
								// abort substate; only used when entry is invalid (NOT a digit)
								// note that this results in TWO writes within a single pass through the FSM (??), which may not work correctly...

								AddDisplayStr("\t\x01B[31m Not a valid DECIMAL entry\x01B[30m");
								DisplayStr();
								fSubStateStatus = SUBSTATE_DONE;					// we are done with the state
								break;
							}
							else if ((eParameterType == PARAMETER_TYPE_BCD) && (isdigit(pgcInputBuffer[bInputBufferIndex]) == ZERO))
							{
								// allow digits
								// abort substate; only used when entry is invalid (NOT a digit)
								// note that this results in TWO writes within a single pass through the FSM (??), which may not work correctly...

								AddDisplayStr("\t\x01B[31m Not a valid BCD entry\x01B[30m");
								DisplayStr();
								fSubStateStatus = SUBSTATE_DONE;					// we are done with the state
								break;
							}
							else if ((eParameterType == PARAMETER_TYPE_HEX) && (isxdigit(pgcInputBuffer[bInputBufferIndex]) == ZERO))
							{
								// allow hex digits
								// abort substate; only used when entry is invalid (NOT a hex digit)
								// note that this results in TWO writes within a single pass through the FSM (??), which may not work correctly...
								AddDisplayStr("\t\x01B[31m Not a valid HEX entry\x01B[30m");
								DisplayStr();
								fSubStateStatus = SUBSTATE_DONE;					// we are done with the state
								break;
							}
							else if ((eParameterType == PARAMETER_TYPE_FLOATING_PT) && (isdigit(pgcInputBuffer[bInputBufferIndex]) == ZERO) && (pgcInputBuffer[bInputBufferIndex] != ASCII_PERIOD) && (pgcInputBuffer[bInputBufferIndex] != ASCII_MINUS))
							{
								// allow digits, period, minus sign
								// abort substate; only used when entry is invalid (NOT a digit && NOT '.' AND NOT '-')
								// note that this results in TWO writes within a single pass through the FSM (??), which may not work correctly..
								AddDisplayStr("\t\x01B[31m Not a valid floating point entry\x01B[30m");
								DisplayStr();
								fSubStateStatus = SUBSTATE_DONE;					// we are done with the state
								break;
							}

							// check for buffer overrun
							else if (bInputBufferIndex == INPUT_STR_SIZE)
							{
								// input buffer is full
								++bInputBufferIndex;								// bump index to point at terminator

								pgcInputBuffer[bInputBufferIndex] = SZ_TERM;		// terminate input string

								++bMenuFSMSequenceCtr;								// force next substate on next FSM entry for processing of parameter
							}
							else
							{
								// accept character and stay in state and substate
								++bInputBufferIndex;								// bump index
							}

							// we may want to add another substate here, to wait for any pending display to complete
						}
						// else stay in state and substate to collect more keystrokes
						break;

					case 2:
						if (bInputBufferIndex > 0)									// <sek> 22 Sep 13 check for meaningful input
						{
							// update parameter
							bValueValid = UpdateParameter(eParameter, pgcInputBuffer);	// update selected parameter based on contents of pgcInputBuffer[]
							if (bValueValid == FALSE)
							{
								AddDisplayStr("\t\x01B[31m Not a valid value\x01B[30m");
								DisplayStr();
							}
						}

						fSubStateStatus = SUBSTATE_DONE;			// we are done with the state
						break;

					default:
						RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
						break;
					}

					break;

					case ST_MENU_DISPLAY_STREAMING_DATA:
						switch(bMenuFSMSequenceCtr)
						{
						case 0:
							// first entry into state, so start display result
							++bMenuFSMSequenceCtr;					// force next substate on next FSM entry
							if (DisplayStatus() IS_FALSE)		// returns FALSE if nothing to display
							{
								fSubStateStatus = SUBSTATE_DONE;	// force exit from state; no need for 2nd substate
							}
							break;

						case 1:
							// all subsequent entries into the state
							// check for status display (RS-232 transmission) complete
//							if(uart_tx_complete() IS_TRUE)
//							{
								++bMenuFSMSequenceCtr;					// force next substate on next FSM entry
								fSubStateStatus = SUBSTATE_DONE;
//							}
							break;

						default:
							RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
							break;
						}
						break;


						case ST_MENU_CHECK_FOR_TERMINATING_KEYSTROKE:
							switch(bMenuFSMSequenceCtr)
							{
							case 0:

								++bMenuFSMSequenceCtr;

							case 1:
								// check for available keystroke
								if(uart_recv(inputbuffer,1) == HAL_OK)
								{
									pgcInputBuffer[0] = inputbuffer[0];
									fSelection = MENU_SELECTED;

									// if we are displaying the Runtime Error status, a keystroke stops the display, and clears the buffer
									if (eCurrentStatus == RUNTIME_ERROR_STATUS)
									{
										ResetRuntimeError();				// reset the buffer pointer; has the effect of clearing the Runtime Error buffer
									}
								}
								else
								{
									fSelection = NOTHING_SELECTED;			// no selection made
								}

								++bMenuFSMSequenceCtr;						// force next (invalid) substate on next FSM entry
								fSubStateStatus = SUBSTATE_DONE;			// keystroke or not, we are done with the state
								break;

							default:
								RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
								break;
							}
							break;

							case ST_MENU_DISPLAY_REALTIME_DATA:						// NOTE: not fully implemented
								switch(bMenuFSMSequenceCtr)
								{
								case 0:
									// enable realtime serial data display
									// ==> this should be conditional on current eSerialOutputMode; if SER_MODE_REMOTE, should NOT change to SER_MODE_REALTIME
									bRealTimeDisplay = TRUE;
									eSerialOutputMode = SER_MODE_REALTIME;


									++bMenuFSMSequenceCtr;						// force next substate on next FSM entry
									break;

								case 1:
									// check for available keystroke


									// a keystroke is available
									if(uart_recv(inputbuffer,1) == HAL_OK)
									{
										pgcInputBuffer[0] = inputbuffer[0];
										fSelection = MENU_SELECTED;

										// any keystroke ends the Realtime Data streaming and returns to the menu
										// disable realtime serial data display
										bRealTimeDisplay = FALSE;
										eSerialOutputMode = SER_MODE_MENU;

										// we may want to add another substate here, to wait for any pending display to complete

										++bMenuFSMSequenceCtr;						// force next (invalid) substate on next FSM entry
										fSubStateStatus = SUBSTATE_DONE;			// keystroke or not, we are done with the state
									}
									else
									{
										fSelection = NOTHING_SELECTED;			// no selection made

										// stay in state and substate
									}
									break;

								default:
									RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
									break;
								}
								break;

								case ST_MENU_DISPLAY_RESULT:
									switch(bMenuFSMSequenceCtr)
									{
									case 0:
										// first entry into state (or re-entry for a new line), so start display result
										if (eResultLine == RESULT_SINGLE_LINE)			// just one line to display
										{
											DisplayStr();								// single line is already setup for display
										}
										// must be line 1 of 1 to n lines..
										else if (DisplayResult() IS_FALSE)				// returns FALSE if nothing MORE to display
										{
											fSubStateStatus = SUBSTATE_DONE;			// force exit from state; no need for 2nd substate
										}

										++bMenuFSMSequenceCtr;							// force next substate on next FSM entry
										break;

									case 1:
										// subsequent entry into the state
										// check for result display (RS-232 transmission) complete
//										if (uart_tx_complete() IS_TRUE)
//										{
											//	if (eResultLine == RESULT_SINGLE_LINE)			// just one line to display
											//	{
											fSubStateStatus = SUBSTATE_DONE;			// force exit from state
											//}

											//bMenuFSMSequenceCtr = 0;						// return to previous substate on next FSM entry
											//++fgwDisplayLineCtr;							// bump to next display line
//										}
										break;

									default:
										RuntimeError(MENU_FSM_ERROR_INVALID_SUBSTATE);
										break;

									}
									break;

									default:
										RuntimeError(MENU_FSM_ERROR_INVALID_STATE);
										break;
	}

	// Note that we do not check for unprocessed events here.. because the FSM is not setup to handle externally generated events.

}


// *****************************************************************************
//							D i s p l a y M e n u ( ) 
// *****************************************************************************

// Display of multi-line Menu Text
// Displays just ONE line per pass through the function, so multiple passes are required.

// returns	TRUE is a line was displayed
//			FALSE if the fgwDisplayLineCtr value goes beyond the last line
BOOL DisplayMenu(void)
{

	BOOL bRetVal = TRUE;
	ClearDisplayStr();

	// check for done with menu text
	if (strlen(fgpMenuText[fgwDisplayLineCtr]) == (size_t)0)
	{
		//    Xbee_Frames(xbee_pkt,"end",3,Coor_Addr);
		// this is actually an error condition...
		eMenuLine = MENU_LAST_LINE;						// this is (effectively) the last line to display
		return (FALSE);									// nothing to display
	}

	// add a line of menu to display
	AddDisplayStr(fgpMenuText[fgwDisplayLineCtr]);

	// bump pointer to menu text to next line
	fgwDisplayLineCtr++;

	// check for done with menu text
	if (strlen(fgpMenuText[fgwDisplayLineCtr]) == (size_t)0)
	{
		eMenuLine = MENU_LAST_LINE;						// this is (effectively) the last line to display
		bRetVal = FALSE;								// nothing to display
	}
	else
	{
		eMenuLine = MENU_LINE;							// at least one more line to display
	}


	AddDisplayNewLine();								// add line terminator

	DisplayStr();									// start display (serial output) of line
	return bRetVal;										// TRUE if more data to display, FALSE if last line
}


// *****************************************************************************
//								M a i n M e n u ( )
// *****************************************************************************
extern FlagStatus debug_cmd;
int MainMenu(char cKeystroke)
{
	int nRetVal = COMMAND_SELECTED;
	int nStatus;
	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	RTCC_DATE_TIME CurrentDateTime;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
	LOCAL SmartTrakOrientation SPAOrientation;
	//	BYTE ucPrevTrackingMode;
	float el=0;
	char temp[20];
	char szReturnStr3[DISPLAY_LINE_SIZE + 1];
	switch(cKeystroke)
	{
	case '0':
		break;
	case '1':				// Run UP
#ifdef USE_ELEVATION
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_UP_SWITCH_OPEN_EVENT);				// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run UP (Forward)");
			memset(dispstr,0,sizeof(dispstr));
			strcpy(dispstr,"Run UP (Forward)");
			////////							HAL_UART_Transmit(UARTid,(unsigned char *)dispstr,strlen(dispstr),10);
			//							while (app_uart_put((uint8_t)dispstr) != NRF_SUCCESS);
			//		app_uart_put((uint8_t)dispstr);
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run UP only available in MANUAL mode");
			memset(dispstr,0,sizeof(dispstr));
			strcpy(dispstr,"Run UP only available in MANUAL mode");
			////////					HAL_UART_Transmit(UARTid,(unsigned char *)dispstr,strlen(dispstr),10);
			//		while (app_uart_put((uint8_t)dispstr) != NRF_SUCCESS);
			//		app_uart_put((uint8_t)dispstr);
			DisplayStr();

		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();

#endif
		break;

	case '2':				// Run EAST
#ifdef USE_AZIMUTH
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run EAST (Reverse)");
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run EAST only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '3':				// Run WEST
#ifdef USE_AZIMUTH
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run WEST (Forward)");
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run WEST only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '4':				// Run DOWN
#ifdef USE_ELEVATION
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run DOWN (Reverse)");
			DisplayStr();
			nRetVal = COMMAND_SELECTED;
		}
		else
		{
			AddDisplayStrAndNewLine("Run DOWN only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '5':				// Find End Points
		BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_CLOSED_EVENT);	// CLOSED is processed first, same event as button push
		BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_OPEN_EVENT);
		AddDisplayStrAndNewLine("Calibrate (Find End Points)");
		DisplayStr();
		nRetVal = COMMAND_SELECTED;
		break;

	case '6':				// Set Current Orientation to Sun Position
		// this is implemented by calculating the offset from the Mechanical Orientation to the SPA Orientation
		DisplayMessage("Current Orientation = Sun Position\r\n", WAIT_FOR_DISPLAY);

		// get, display current date and time
		if(ReadRTCCDateTime(ptrDateTime) != TRUE)
		{
			DisplayMessage("Unable to Read RTCC\r\n", WAIT_FOR_DISPLAY);
		}
		IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
		DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);
		// calculate and display Sun Position, global coordinates
		/*fgSPAMove.SunPositionState = */ IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
		strcpy(szfnDisplayStr, "SPA ");
		Orientation_Format((szfnDisplayStr + strlen(szfnDisplayStr)), &SPAOrientation);	// format for display
		DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);

		// convert SPA orientation to local orientation
		ConvertSPAtoLocalOrientation(&SPAOrientation);

		// display new Local orientation
		strcpy(szfnDisplayStr, "Local ");
		Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);	// format for display
		AddDisplayStrAndNewLine(szfnDisplayStr);

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

		nRetVal = COMMAND_SELECTED;
		break;

	case '7':				// fAZ_Offset
		memset(dispstr,0,sizeof(dispstr));
		AddDisplayStr("fAZ_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_Offset, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		DisplayStr();
		break;

	case '8':				// fEL_Offset
#ifdef USE_ELEVATION
		AddDisplayStr("fAZ_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_Offset, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		DisplayStr();										// start display (serial output) of line
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '9':				// Current Local Orientation
#ifdef USE_MMA8452Q_INCLINOMETER
		// re-Initialize Inclinometer Averaging by refilling the averaging array (twice).
		if (Init_InclinometerSampleAveraging() != TRUE)
		{
			DisplayMessage("Unable to Read Inclinometer\r\n", WAIT_FOR_DISPLAY);
		}
#endif	//  USE_MMA8452Q_INCLINOMETER
		AddDisplayStr("Current Local Orientation (degrees): ");
		CurrentLocalOrientation_Format(szfnDisplayStr);
		AddDisplayStrAndNewLine(szfnDisplayStr);
		DisplayStr();
		nRetVal = COMMAND_SELECTED;

		break;

	case 'a':				// SPA Calculated Orientation
	{
		if (ReadRTCCDateTime(ptrDateTime) != TRUE)
		{
			//  RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
			strcpy(szReturnStr3, "0:0:0:0:0:0:0:0:0:0\n\r");
			DisplayMessage( szReturnStr3, WAIT_FOR_DISPLAY);
			return FALSE;
		}
		else
		{
			ADD2RTCCDateTime(szReturnStr3, ptrDateTime);
		}
		strcat(szReturnStr3, ":");
		IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
		el=SPAOrientation.fElevation;
		ConvertSPAtoLocalOrientation(&SPAOrientation);
		strcat(szReturnStr3,(const char *)ftoa2(SPAOrientation.fAzimuth, &nStatus));
		strcat(szReturnStr3, ":");
		if(ptrRAM_SystemParameters->ucTracking_Mode == MODE_TRACKING)
		{
			IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
			PanelPositionFSM(&SPAOrientation);
			strcat(szReturnStr3,(const char *)ftoa(SPAOrientation.fAzimuth , &nStatus));
			strcat(szReturnStr3, ":");
		}
		else
		{
			if((MAN_WEST == 1)||(MAN_EAST == 1))
			{
				strcat(szReturnStr3,(const char *)ftoa(pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
				strcat(szReturnStr3, ":");
			}
			else
			{
				//strcat(szReturnStr,);
				strcat(szReturnStr3, "0.0:");
			}
		}

#ifdef USE_MMA8452Q_INCLINOMETER
		// strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
		// strcat(szReturnStr, ":");

		strcat(szReturnStr3, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
		strcat(szReturnStr3, ":");
#endif

		//IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr + strlen(szReturnStr), ptrDateTime);
		//strcat(szReturnStr, ":");
		//4digits    x1x2x3x4
		//    x1- 0POWER_UP/1INIT/2TRACKING/3HOLD/4NIGHT_STO/5WIND_STOW
		//    x2- 1MAN/2AUTO     x3- 0stopped/1West/2East
		//    x4- 0MotorStooped/1Motor running
		BYTEtoASCIIstr(SPS, temp);
		if(((int)SPS < 1) || ((int)SPS > 6))
			strcat(szReturnStr3,"07");
		else
			strcat(szReturnStr3, temp);
		if(ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			if(MAN_EAST == 1)
				strcat(szReturnStr3, "12");  //east
			else if(MAN_WEST == 1)
				strcat(szReturnStr3, "11");  //West
			else if(MAN_STOW == 1)
				strcat(szReturnStr3, "13");  //stow
			else
				strcat(szReturnStr3, "10");
		}
		else
		{
			if(bTrackerDirection == 1)
				strcat(szReturnStr3, "21");  //West
			else if(bTrackerDirection == 2)
				strcat(szReturnStr3, "22");  //east
			else
				strcat(szReturnStr3, "20");  //stop
		}
		if (IsCommandComplete(AXIS_AZIMUTH) IS_TRUE)
			strcat(szReturnStr3, "0\n\r");
		else
			strcat(szReturnStr3, "1\n\r");
	}
	DisplayMessage(szReturnStr3, WAIT_FOR_DISPLAY);
	break;

	case 'b':				// clear Orientation
		AddDisplayStrAndNewLine("Clear Current Orientation from RTCC NV RAM and MCU RAM Copy");
		CurrentPosition_Clear(MOTOR_AZIMUTH);								// clear the MSI Tick counters
		CurrentPosition_Clear(MOTOR_ELEVATION);
		// ClearRTCCRAMOrientation() handles #ifdef USE_DS3232_RTCC internally
		IGNORE_RETURN_VALUE ClearRTCCRAMOrientation();						// clear MCU RAM copy of Orientation, then Update the RTCC NV RAM copy of Orientation
		nRetVal = COMMAND_SELECTED;
		DisplayStr();
		break;

	case 'c':				// clear Orientation
#ifdef USE_DS3232_RTCC
		AddDisplayStrAndNewLine("Clear Entire RTCC NV RAM, Init MCU RAM Copy");
		DisplayStr();
		IGNORE_RETURN_VALUE ClearRTCCRAM();
#else
		AddDisplayStrAndNewLine("RTCC not available. Init MCU RAM Copy ONLY");
		DisplayStr();
#endif
		// clear RTCC NV RAM parameter table, current orientation
		// InitRTCCRAMParameterTable() handles #ifdef USE_DS3232_RTCC internally
/*		if(InitRTCCRAMParameterTable() != MEMORY_INITIALIZED)			// re-initialize tables (orientation will be 0, 0)
		{
			AddDisplayStrAndNewLine("RAM Initialization Failure");
			DisplayStr();
		}*/
		nRetVal = COMMAND_SELECTED;
		break;

	case 'd':				// ucTracking_Mode
		BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
		BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
		AddDisplayStrAndNewLine("Auto/Manual Mode");
		nRetVal = COMMAND_SELECTED;
		AddDisplayStr("Prev ucTracking_Mode: ");
		//			ucPrevTrackingMode = ptrRAM_SystemParameters->ucTracking_Mode;		// keep track of previous tracking mode
		BYTEtoHexASCIIstr(ptrRAM_SystemParameters->ucTracking_Mode, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		switch(ptrRAM_SystemParameters->ucTracking_Mode)
		{
		case MODE_MANUAL:
			AddDisplayStr(" Manual Mode");
			break;

		case MODE_TRACKING:
			AddDisplayStr(" AUTO SPA Tracking");
			break;

		case MODE_NIGHT_STOW:
		case MODE_WIND_STOW:
		default:
			AddDisplayStr(" Unknown/Error");
			break;
		}
		DisplayStr();										// start display (serial output) of line
		break;

		case 'e':				// write parameter table to flash
			WriteFlashParameterTable();		// ==> should have a return value
			AddDisplayStrAndNewLine("Table Write Complete");
			DisplayStr();
			break;

		case 'f':				// display runtime errors
			eCurrentStatus = RUNTIME_ERROR_STATUS;
			fgwErrorDisplayIndex = 0;					// reset error display index
			nRetVal = DATA_STREAM_SELECTED;
			break;

		case 'g':				// Utility Menu
			eCurrentMenu = UTILITY_MENU;
			nRetVal = MENU_SELECTED;
			break;

		case 'h':				// Remote Mode
			if(debug_cmd == SET)
				debug_cmd = RESET;
			else
				debug_cmd = SET;
			AddDisplayStrAndNewLine("\x1B[2J\x1B[0;0H Remote Mode: ");
			DisplayStr();										// start display (serial output) of line

			ptrRAM_SystemParameters->eSerialOutputMode = SER_MODE_REMOTE;
			eSerialOutputMode = SER_MODE_REMOTE;								// update program global copy
			////			ChangeSerialBaudRate(SERIAL_REMOTE_UART, DESIRED_REMOTE_BAUDRATE);
			WriteFlashParameterTable();
			//			nRetVal = PARAMETER_SELECTED;
			//			eParameter = PARAMETER_AZ_OFFSET;
			//			eParameterType = PARAMETER_TYPE_DEC;
			break;

		case 'i':				// Activate internal accelerometer
			gIsMma845Enabled = FALSE;
			LIS2HH12_Init();
			if (Init_InclinometerSampleAveraging() IS_TRUE)
			{
				// read and display inclinometer angles
				if (ReadInclinometerSample(&pgInclination) == TRUE)							// read accelerometer, calculate 3D angles
				{
					IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);// add to averaging array, calculate new running average
				}
				// initialze previous position to stored AVERAGED value, before allowing ANY motion
				fglPreviousPosition[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
				CurrentPosition_Set(MOTOR_AZIMUTH, fglPreviousPosition[MOTOR_AZIMUTH]);
			}
			break;

		case 'j':				// Activate internal accelerometer
			gIsMma845Enabled = TRUE;
			MMA845x_Init();
			if (Init_InclinometerSampleAveraging() IS_TRUE)
			{
				// read and display inclinometer angles
				if (ReadInclinometerSample(&pgInclination) == TRUE)							// read accelerometer, calculate 3D angles
				{
					IGNORE_RETURN_VALUE AverageInclinometerSample(&pgInclination);// add to averaging array, calculate new running average
				}
				// initialze previous position to stored AVERAGED value, before allowing ANY motion
				fglPreviousPosition[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(pgAngleAverage.fX_Angle, AXIS_AZIMUTH);
				CurrentPosition_Set(MOTOR_AZIMUTH, fglPreviousPosition[MOTOR_AZIMUTH]);
			}
			break;
		case 'k':
			AddDisplayStr("Internal IMU Sensor Test...");
			DisplayStr();
			break;

		case 's':				// stop
			// make sure there is an active motor to stop
			if (IsMoveSequenceComplete() IS_FALSE)
			{
				// there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
				BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
				// no need for an OPEN even, STOP clears all switch and virtual switch events
				AddDisplayStrAndNewLine("All STOP!");
				DisplayStr();
				nRetVal = COMMAND_SELECTED;
			}
			break;

		case 'r':				// reset
			BITSET(efSwitchEvents, EF_SW_RESET_SWITCH_CLOSED_EVENT);
			break;

	}

	eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
	return nRetVal;

}



// *****************************************************************************
//								U t i l i t y M e n u ( )
// *****************************************************************************

int UtilityMenu(char cKeystroke)
{

	switch(cKeystroke)
	{
	case '1':				// Status Menu
		eCurrentMenu = STATUS_MENU;
		break;

	case '2':				// Motor Settings
		eCurrentMenu = MOTION_SETTINGS_MENU;
		break;

	case '3':				// Movement
		eCurrentMenu = OPEN_LOOP_MOVE_MENU;
		break;

	case '4':				// Movement
		eCurrentMenu = CLOSED_LOOP_MOVE_MENU;
		break;

	case '5':				// Movement
		eCurrentMenu = MOVE_SEQUENCE_MENU;
		break;

	case '6':				// Movement
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		break;

	case '7':				// Read Inputs
		eCurrentMenu = READ_INPUTS_MENU;
		break;

	case '8':				// Read Inputs
		eCurrentMenu = TEST_MENU;
		break;

#ifdef USE_PGBTERMINATE
	case '8':				// Restart system
		pgbTerminate = TRUE;	// set flag to restart
		break;
#endif

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = MAIN_MENU;
		break;

	default:
		RuntimeError(MENU_FSM_ERROR_INVALID_STATE);
		// redisplay current menu
		break;
	}

	return MENU_SELECTED;

}

// *****************************************************************************
//								S t a t u s M e n u ( ) 
// *****************************************************************************

int StatusMenu(char cKeystroke)
{

	int nRetVal = COMMAND_SELECTED;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// last movement
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		eResultType = RESULT_STATUS_LAST_EL_MOVEMENT;
		nRetVal = COMMAND_SELECTED;
		break;

	case '2':				// last movement
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		eResultType = RESULT_STATUS_LAST_AZ_MOVEMENT;
		nRetVal = COMMAND_SELECTED;
		break;

	case '3':				// stream input switch status data
#ifdef MOTION_ERROR_TABLE
		eCurrentStatus = MOTION_ERROR_STATUS;
		fgwMotionErrorDisplayIndex = 0;				// reset error display index (was MIN_SPEED_INDEX)
		nRetVal = DATA_STREAM_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
		nRetVal = MENU_SELECTED;
#endif
		break;

	case '4':				// stream FSM States
#ifdef USE_ELEVATION
		eCurrentStatus = EL_FSM_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
#endif
		break;

	case '5':				// stream FSM States
#ifdef USE_AZIMUTH
		eCurrentStatus = AZ_FSM_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
#endif
		break;

	case '6':				// stream motor status data
		eCurrentStatus = MOTOR_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
		break;

	case '7':				// stream input switch status data
		eCurrentStatus = INPUT_SWITCH_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
		break;

	case '8':				// stream inclinimeter readings (direct)
#ifdef USE_MMA8452Q_INCLINOMETER
		eCurrentStatus = INCLINOMETER_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
		nRetVal = MENU_SELECTED;
#endif
		break;

	case '9':				// stream average of inclinimeter readings
#ifdef USE_MMA8452Q_INCLINOMETER
		eCurrentStatus = INCLINOMETER_AVERAGE_STATUS;
		nRetVal = DATA_STREAM_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
		nRetVal = MENU_SELECTED;
#endif
		break;

	case 'a':				// stream average of inclinimeter readings
#ifdef USE_SINGLE_POLAR_AXIS
		eCurrentStatus = POLAR_AXIS_MOVE_STATUS;
		fgbPolarAxisMoveDisplayIndex = 0;				// reset Polar Axis move display index
		nRetVal = DATA_STREAM_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
		nRetVal = MENU_SELECTED;
#endif
		break;

	case 'b':				// stream average of inclinimeter readings
		eCurrentStatus = SUN_ANGLES_STATUS;
		fgbStreamSunAnglesState = 0;					// reset Sun Angle display substate counter
		nRetVal = DATA_STREAM_SELECTED;
		break;
	case 'c':				// stream average of inclinimeter readings
		eCurrentStatus = BMS_STATUS;
		fgbStreamSunAnglesState = 0;					// reset Sun Angle display substate counter
		nRetVal = DATA_STREAM_SELECTED;
		break;

	case 'e':				// stream runtime error data
		eCurrentStatus = RUNTIME_ERROR_STATUS;
		fgwErrorDisplayIndex = 0;					// reset error display index
		nRetVal = DATA_STREAM_SELECTED;
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}

	return nRetVal;
}

// *****************************************************************************
//						M o t o r S e t t i n g s M e n u ( ) 
// *****************************************************************************

int MotionSettingsMenu(char cKeystroke)
{

	//	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_MTN_SENSOR_SETTINGS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '2':
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_PWM_SETTINGS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '3':				// PWM widths
		// multi-line output display
		eResultType = RESULT_MOTOR_PWM_WIDTHS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

#ifdef USE_PWM_DUTY_CYCLE
	case '4':
		AddDisplayStr("Elevation Duty Cycle: ");

		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_ELEVATION], szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_ELEVATION_DUTY_CYCLE;
		eParameterType = PARAMETER_TYPE_DECIMAL;
		break;

	case '5':
		AddDisplayStr("Azimuth Duty Cycle: ");
		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_AZIMUTH], szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZIMUTH_DUTY_CYCLE;
		eParameterType = PARAMETER_TYPE_DECIMAL;
		break;
#endif

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}

	return nRetVal;
}


// *****************************************************************************
//					O p e n L o o p M o v e M e n u ( )
// *****************************************************************************

// Note: these menu selections are LOW LEVEL operations intended for development and early hardware testing.
// They directly set the PWM and H-Bridge configuration, and DO NOT use the MotionPhaseFSM or MotionFSM.

int OpenLoopMoveMenu(char cKeystroke)
{

	//	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];

	int nRetVal = COMMAND_SELECTED;

	// clear the screen
	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	// each of these cases simply sets a menu event flag for the requested move
	// the event flags are subsequently processed by the User FSM
	case '1':				// elevation duty cycle
#ifdef USE_PWM_DUTY_CYCLE
		AddDisplayStr("Elevation Duty Cycle: ");
		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_ELEVATION], szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_ELEVATION_DUTY_CYCLE;
		eParameterType = PARAMETER_TYPE_DECIMAL;
#else
		AddDisplayStr("Not Available");
		DisplayStr();
#endif
		break;

	case '2':				// run up
		AddDisplayStrAndNewLine("Run up (fwd)");
		DisplayStr();
#ifdef USE_ELEVATION_ON_OFF_LINEAR_DRIVE
		SetJ9Pin(PIN_MOTOR_REV_RELAY);						// relay drive for 2 x 5V Relay board is active LOW
		ClearJ9Pin(PIN_MOTOR_FWD_RELAY);
#else
//		EnableMotorDrive();									// enable H-Bridge, needed here because OpenLoop moves do NOT call MotionFSM()
		PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_STOPPED);	// must stop before changing directions
		// ==>> need a delay here
		PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_FORWARD);
		IGNORE_RETURN_VALUE PWM_SetDutyCycle(MOTOR_ELEVATION, PWM_DUTY_CYCLE_MIN);
		pgeMotionPhase[MOTOR_ELEVATION] = PHASE_OPEN_LOOP;	// allows MSI tick counting WITHOUT any speed adjustment
#ifdef USE_FEEDBACK_SIMULATOR
		PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_ELEVATION, PWM_DUTY_CYCLE_MIN);		// set MotionProfileSpeedAndPWM[] index for simulator. matching current PWM value
		MS_SetSimulatorTickCtr(MOTOR_ELEVATION);		// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
#endif
#endif
		break;

	case '3':				// run down
		AddDisplayStrAndNewLine("Run down (rev)");
		DisplayStr();
#ifdef USE_ELEVATION_ON_OFF_LINEAR_DRIVE
		SetJ9Pin(PIN_MOTOR_FWD_RELAY);						// relay drive for 2 x 5V Relay board is active LOW
		ClearJ9Pin(PIN_MOTOR_REV_RELAY);
#else
//		EnableMotorDrive();									// enable H-Bridge, needed here because OpenLoop moves do NOT call MotionFSM()
		PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_STOPPED);	// must stop before changing directions
		// ==>> need a delay here
		PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_REVERSE);
		IGNORE_RETURN_VALUE PWM_SetDutyCycle(MOTOR_ELEVATION, PWM_DUTY_CYCLE_MIN);
		pgeMotionPhase[MOTOR_ELEVATION] = PHASE_OPEN_LOOP;	// allows MSI tick counting WITHOUT any speed adjustment
#ifdef USE_FEEDBACK_SIMULATOR
		PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_ELEVATION, PWM_DUTY_CYCLE_MIN);		// set MotionProfileSpeedAndPWM[] index for simulator. matching current PWM value
		MS_SetSimulatorTickCtr(MOTOR_ELEVATION);		// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
#endif
#endif
		break;

	case '4':
#ifdef USE_PWM_DUTY_CYCLE
		AddDisplayStr("Azimuth Duty Cycle: ");
		BYTEtoASCIIstr(pgbPWMDutyCycle[MOTOR_AZIMUTH], szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		DisplayStr(UARTid);									// start display (serial output) of line
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZIMUTH_DUTY_CYCLE;
		eParameterType = PARAMETER_TYPE_DECIMAL;
#endif
		break;

	case '5':
		AddDisplayStrAndNewLine("Run right (fwd)");
		DisplayStr();
//		EnableMotorDrive();									// enable H-Bridge, needed here because OpenLoop moves do NOT call MotionFSM()
		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);	// must stop before changing directions
		// ==>> need a delay here
		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_FORWARD);	// also enables motion sensor interrupts
		IGNORE_RETURN_VALUE PWM_SetDutyCycle(MOTOR_AZIMUTH, PWM_DUTY_CYCLE_MIN);
		pgeMotionPhase[MOTOR_AZIMUTH] = PHASE_OPEN_LOOP;	// allows MSI tick counting WITHOUT any speed adjustment
#ifdef USE_FEEDBACK_SIMULATOR
		PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_AZIMUTH, PWM_DUTY_CYCLE_MIN);		// set MotionProfileSpeedAndPWM[] index for simulator. matching current PWM value
		MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);			// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
#endif
		break;

	case '6':
		AddDisplayStrAndNewLine("Run left (rev)");
		DisplayStr();
//		EnableMotorDrive();									// enable H-Bridge, needed here because OpenLoop moves do NOT call MotionFSM()
		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);	// must stop before changing directions
		// ==>> need a delay here
		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_REVERSE);	// also enables motion sensor interrupts
		IGNORE_RETURN_VALUE PWM_SetDutyCycle(MOTOR_AZIMUTH, PWM_DUTY_CYCLE_MIN);
		pgeMotionPhase[MOTOR_AZIMUTH] = PHASE_OPEN_LOOP;	// allows MSI tick counting WITHOUT any speed adjustment
#ifdef USE_FEEDBACK_SIMULATOR
		PWM_FindMotionProfileSpeedAndPWMIndex(MOTOR_AZIMUTH, PWM_DUTY_CYCLE_MIN);		// set MotionProfileSpeedAndPWM[] index for simulator. matching current PWM value
		MS_SetSimulatorTickCtr(MOTOR_AZIMUTH);			// sets simlator Motion Sensor tick counter according to current value of pgbMotionProfileSpeedIndex[eMotor]
#endif
		break;

	case 's':				// ALL STOP
		AddDisplayStrAndNewLine("All Stop!");
		DisplayStr();
#ifdef USE_ELEVATION_ON_OFF_LINEAR_DRIVE
		SetJ9Pin(PIN_MOTOR_FWD_RELAY);				// relay drive for 2 x 5V Relay board is active LOW
		SetJ9Pin(PIN_MOTOR_REV_RELAY);
#else
		PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_STOPPED);	// stop both axis, clears saved PWM value to 0
#endif
		PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
//		DisableMotorDrive();								// disable H-Bridge, needed here because OpenLoop moves do NOT call MotionFSM()
		pgeMotionPhase[MOTOR_AZIMUTH] = PHASE_STOPPED;
		pgeMotionPhase[MOTOR_ELEVATION] = PHASE_STOPPED;
		pgeMotionType[MOTOR_AZIMUTH] = MOTION_STOPPED;
		pgeMotionType[MOTOR_ELEVATION] = MOTION_STOPPED;
		break;

	case 'r':				// reset
		////			Reset();													// processor reset
		//lint -fallthrough  no need for a break here, we are doing a hard reset

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}

	eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
	return nRetVal;
}


// *****************************************************************************
//					C l o s e d L o o p	M o v e M e n u ( )
// *****************************************************************************


int ClosedLoopMoveMenu(char cKeystroke)
{

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nStatus = 0;
	int nRetVal = COMMAND_SELECTED;

	// clear the screen
	ClearDisplayStr();
	AddDisplayNewLine();

	// display the current panel orientation
	AddDisplayStr("\tOrientation: ");
	CurrentMechanicalOrientation_Format(szfnDisplayStr);
	AddDisplayStrAndNewLine(szfnDisplayStr);
	DisplayStr();
	ClearDisplayStr();

	// if the keystroke is 's', it can be processed to stop the current operation (some can be very long!)
	if ((cKeystroke != 's') && (cKeystroke != 'r') && (cKeystroke != 'x'))
	{
		// check for completion of previously selected command
		// if a command is still in process, STOP is the only new command that can be processed
		if (IsCommandComplete(eClosedLoopActiveMotor) IS_FALSE)
		{
			// previous command has not been completed, so do NOT allow selection of additional commands OTHER than Reset
			RuntimeError(MENU_FSM_ERROR_COMMAND_OVERRUN);
			AddDisplayStrAndNewLine("NOT Done! Please wait for completion or s) Stop");
			DisplayStr();

#ifndef _lint				// for some reason, putting this #ifdef around a single line does not work correctly...
			if (cKeystroke == 'r')
			{
				////					Reset();							// forces a complete reset, macro to  {_asm reset _endasm} gets PC-Lint ver lost!
			}
#endif

			eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
			return COMMAND_SELECTED;
		}
		else
		{
			// previous command (if any) must have completed
			eClosedLoopActiveMotor = MOTOR_NONE;
		}
	}

	switch(cKeystroke)
	{
	// each of these cases simply sets a menu event flag for the requested move
	// the event flags are subsequently processed by the User FSM
	case '1':				// set move distace
		AddDisplayStr("Closed Loop Run Distance (AZ or EL):");


		AddDisplayStr((const char *)ftoa2(fgfCLMoveDistance, &nStatus));

		AddDisplayStr("  _");

		DisplayStr();												// start display (serial output) of line

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_MOVE_DISTANCE;							// set fgfCLMoveDistance
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

#ifdef USE_ELEVATION
	case '2':				// move up
		eClosedLoopActiveMotor = MOTOR_ELEVATION;							// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);									// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_RUN_FWD);	// Forward == UP
		pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(fgfCLMoveDistance, AXIS_ELEVATION);
		AddDisplayStr("Run Up ");
		AddDisplayStr((const char *)ftoa2(fgfCLMoveDistance, &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();

		break;

	case '3':				// move down
		eClosedLoopActiveMotor = MOTOR_ELEVATION;							// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);									// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_RUN_REV);	// Reverse == DOWN
		pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(-fgfCLMoveDistance, AXIS_ELEVATION);
		AddDisplayStr("Run Down ");
		AddDisplayStr((const char *)ftoa2(fgfCLMoveDistance, &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();
		break;
#endif	// USE_ELEVATION

	case '4':				// move right
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;								// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);									// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_RUN_FWD);	// Forward == RIGHT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(fgfCLMoveDistance, AXIS_AZIMUTH);
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = fgfCLMoveDistance;
#endif
		AddDisplayStr("Run Right ");
		AddDisplayStr((const char *)ftoa2(fgfCLMoveDistance, &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();
		break;

	case '5':				// move left
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;								// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);									// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_RUN_REV);	// Reverse == LEFT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(-fgfCLMoveDistance, AXIS_AZIMUTH);
#ifdef USE_INCLINOMETER_FEEDBACK
		//				pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = -fgfCLMoveDistance;
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = fgfCLMoveDistance;		// distance is absolute value, direction is set by EF_MTN_CMD_RUN_REV
#endif
		AddDisplayStr("Run Left ");
		AddDisplayStr((const char *)ftoa(fgfCLMoveDistance, &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();

		break;

#ifdef USE_ELEVATION
	case '6':				// move up to EOT
		eClosedLoopActiveMotor = MOTOR_ELEVATION;							// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);									// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_SLEW_FWD_TO_END);	// Forward == UP
		pglMoveDistanceTicks[MOTOR_ELEVATION] = EL_MSI_TICKS_PER_SLEW;
		AddDisplayStrAndNewLine("Slew Up to End (fwd)");
		DisplayStr();

		break;

	case '7':				// move down to EOT
		eClosedLoopActiveMotor = MOTOR_ELEVATION;									// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_SLEW_REV_TO_END);	// Reverse == DOWN
		pglMoveDistanceTicks[MOTOR_ELEVATION] = -EL_MSI_TICKS_PER_SLEW;
		AddDisplayStrAndNewLine("Slew Down to End (rev)");
		DisplayStr();

		break;
#endif	// USE_ELEVATION

	case '8':				// move right to EOT
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;									// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_SLEW_FWD_TO_END);	// Forward == RIGHT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = AZ_MSI_TICKS_PER_SLEW;
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = AZ_DEGREES_PER_SLEW;
#endif
		AddDisplayStrAndNewLine("Slew Right to End (fwd)");
		DisplayStr();

		break;

	case '9':				// move left to EOT
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;									// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_SLEW_REV_TO_END);	// Reverse == LEFT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = -AZ_MSI_TICKS_PER_SLEW;
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = -AZ_DEGREES_PER_SLEW;
#endif

		AddDisplayStrAndNewLine("Slew Left to End (rev)");
		DisplayStr();

		break;

#ifdef USE_ELEVATION
	case 'a':				// one Motor Gearbox rotation up
		eClosedLoopActiveMotor = MOTOR_ELEVATION;									// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_RUN_FWD);		// Forward == UP
		pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(EL_MOTOR_GB_ROTATION_DEGREES, MOTOR_ELEVATION);
		AddDisplayStr("Run One Rotation up (fwd)  ");
		AddDisplayStr((const char *)ftoa((EL_MOTOR_GB_ROTATION_DEGREES), &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();

		break;

	case 'b':				// one Motor Gearbox rotation down
		eClosedLoopActiveMotor = MOTOR_ELEVATION;									// keep track of active motor
		SetCommandStarted(MOTOR_ELEVATION);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_RUN_REV);		// Reverse == DOWN
		pglMoveDistanceTicks[MOTOR_ELEVATION] = ConvertDegreesToMSITicks(-EL_MOTOR_GB_ROTATION_DEGREES, MOTOR_ELEVATION);
		AddDisplayStr("Run One Rotation down (rev)  ");
		AddDisplayStr((const char *)ftoa((-EL_MOTOR_GB_ROTATION_DEGREES), &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();
		break;
#endif	// USE_ELEVATION


	case 'c':				// one  Motor Gearbox rotation right
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;									// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_RUN_FWD);		// Forward == RIGHT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(AZ_MOTOR_GB_ROTATION_DEGREES, MOTOR_AZIMUTH);
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = AZ_MOTOR_GB_ROTATION_DEGREES;
#endif
		AddDisplayStr("Run One Rotation right (fwd)  ");
		AddDisplayStr((const char *)ftoa((AZ_MOTOR_GB_ROTATION_DEGREES), &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();

		break;

	case 'd':				// one  Motor Gearbox rotation left
		eClosedLoopActiveMotor = MOTOR_AZIMUTH;									// keep track of active motor
		SetCommandStarted(MOTOR_AZIMUTH);								// mark command as started so we cannot misinterpret completion
		BITSET(efMotionPhaseCommands[MOTOR_AZIMUTH], EF_MTN_CMD_RUN_REV);		// Reverse == LEFT
		pglMoveDistanceTicks[MOTOR_AZIMUTH] = ConvertDegreesToMSITicks(-AZ_MOTOR_GB_ROTATION_DEGREES, MOTOR_AZIMUTH);
#ifdef USE_INCLINOMETER_FEEDBACK
		pgfMoveDistanceDegrees[MOTOR_AZIMUTH] = -AZ_MOTOR_GB_ROTATION_DEGREES;
#endif
		AddDisplayStr("Run One Rotation left (rev)  ");
		AddDisplayStr((const char *)ftoa((AZ_MOTOR_GB_ROTATION_DEGREES), &nStatus));
		AddDisplayStrAndNewLine(" Degrees");
		DisplayStr();
		break;

	case 's':				// stop
		// make sure there is an active motor to stop
		if (eClosedLoopActiveMotor != MOTOR_NONE)
		{
			Finish_MotionStats(eClosedLoopActiveMotor);
			AddDisplayStr("MenuFSM.c: Line 2578\n");
			// ==>> does the previous command ever get completed?
			SetCommandStarted(eClosedLoopActiveMotor);								// mark command as started so we cannot misinterpret completion
			BITSET(efMotionPhaseCommands[eClosedLoopActiveMotor], EF_MTN_CMD_STOP);	// bring to an orderly stop
			//SetCommandStarted(MOTOR_ELEVATION);								// mark command as started so we cannot misinterpret completion
			//BITSET(efMotionPhaseCommands[MOTOR_ELEVATION], EF_MTN_CMD_STOP);
			ResetMotionFSM();												// prepare MotionFSM to resetart
			AddDisplayStrAndNewLine("All Stop!");
			DisplayStr();
		}
		break;

	case 'r':				// reset
		////			Reset();													// processor reset
		//lint -fallthrough  no need for a break here, we are doing a hard reset

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();

		nRetVal = MENU_SELECTED;
		break;
	}

	eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
	return nRetVal;
}


// *****************************************************************************
//							M o v e S e q u e n c e M e n u ( ) 
// *****************************************************************************

int MoveSequenceMenu(char cKeystroke)
{

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nStatus = 0;
	int nRetVal = COMMAND_SELECTED;

	// clear the screen
	ClearDisplayStr();
	AddDisplayNewLine();

	// display the current panel orientation
	AddDisplayStr("\tLocal Orientation: ");
	CurrentLocalOrientation_Format(szfnDisplayStr);
	AddDisplayStrAndNewLine(szfnDisplayStr);
	DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);
	ClearDisplayStr();

	// if the keystroke is 's', it can be processed to stop the current operation (some can be very long!)
	if ((cKeystroke != 's') && (cKeystroke != 'r') && (cKeystroke != 'x'))
	{
		// check for completion of previously selected command
		// if a command is still in process, STOP is the only new command that can be processed
		if (IsMoveSequenceComplete() IS_FALSE)
		{
			// previous command has not been completed, so do NOT allow selection of additional commands OTHER than Reset
			RuntimeError(MENU_FSM_ERROR_COMMAND_OVERRUN);
			AddDisplayStrAndNewLine("NOT Done! Please wait for completion or s) Stop");
			DisplayStr();
#ifndef _lint				// for some reason, putting this #ifdef around a single line does not work correctly...
			if (cKeystroke == 'r')
			{
				////					Reset();							// forces a complete reset, macro to  {_asm reset _endasm} gets PC-Lint ver lost!
			}
#endif

			eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
			return COMMAND_SELECTED;
		}
	}

	switch(cKeystroke)
	{
	// each of these cases simply sets a menu event flag for the requested move
	// the event flags are subsequently processed by the User FSM
	case '1':				// find end points
		SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_FIND_END_POINTS);	// set flag to start movement sequence
		AddDisplayStrAndNewLine("Finding End Points.. Please wait!");
		DisplayStr();
		break;

	case '2':				// move to (degrees, degrees) position  ==>> needs to be implemented

		SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_POSITION);
		AddDisplayStrAndNewLine("Moving to Degrees, Degrees (NOT IMPLEMENTED)");
		DisplayStr();
		break;

	case '3':				// move to center position
		SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_POSITION);
		AddDisplayStrAndNewLine("Moving to Center (NOT IMPLEMENTED)");
		DisplayStr();

		break;

	case '4':				// move to Night stow position
		SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_NIGHT_STOW);
		AddDisplayStrAndNewLine("Moving to Night Stow (NOT IMPLEMENTED)");
		DisplayStr();
		break;

	case '5':				// move to Wind stow position
		SetMoveSequenceStarted();									// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_MOVE_TO_WIND_STOW);
		AddDisplayStrAndNewLine("Moving to Wind Stow (NOT IMPLEMENTED)");
		DisplayStr();

		break;

	case '6':				// set move distace
		AddDisplayStr("Step Size (degrees): ");
		AddDisplayStr((const char *)ftoa(fgfMSStepSize, &nStatus));
		AddDisplayStr("  _");
		DisplayStr();												// start display (serial output) of line
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_STEP_SIZE;								// set fgfMSStepSize
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '7':				// Calculate Sun Position
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_CALCULATE);
		AddDisplayStrAndNewLine("Starting SPA Calculation (no movement)");
		DisplayStr();

		break;

	case '8':				// move to Sun Tracking Algorithm position
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_SPA_TRACK);
		AddDisplayStrAndNewLine("Starting SPA Track (Calculated Movements)");
		DisplayStr();
		break;

#ifdef USE_ELEVATION
	case '9':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_ELEVATION] = fgfMSStepSize;		// set movement distance
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_UP);
		AddDisplayStrAndNewLine("Run 1 Step Up");
		DisplayStr();
		break;

	case 'a':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_ELEVATION] = fgfMSStepSize;		// set movement distance
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_DOWN);
		AddDisplayStrAndNewLine("Run 1 Step Down");
		DisplayStr();
		break;
#endif	// USE_ELEVATION

#ifdef USE_AZIMUTH
	case 'b':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgfMSStepSize;		// set movement distance
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_RIGHT);
		AddDisplayStrAndNewLine("Run 1 Step Right");
		DisplayStr();
		break;

	case 'c':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgfMSStepSize;		// set movement distance
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_RUN_LEFT);
		AddDisplayStrAndNewLine("Run 1 Step Left");
		DisplayStr();
		break;

	case 'd':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgfMSStepSize;		// set movement distance (degrees)
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_RIGHT);
		AddDisplayStrAndNewLine("Incremental Run 1 Step Right");
		DisplayStr();
		break;

	case 'e':
		SetMoveSequenceStarted();										// mark Move Sequence as started so we cannot misinterpret completion
		pgfMS_StepDistanceDegrees[MOTOR_AZIMUTH] = fgfMSStepSize;		// set movement distance (degrees)
		BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_INC_RUN_LEFT);
		AddDisplayStrAndNewLine("Incremental Run 1 Step Left");
		DisplayStr();
		break;
#endif // USE_AZIMUTH

	case 's':				// stop
		// make sure there is an active motor to stop
		if (IsMoveSequenceComplete() IS_FALSE)
		{
			BITSET(efMoveSequenceEvents, EF_MOVE_SEQ_STOP);				// bring to an orderly STOP
			AddDisplayStrAndNewLine("All Stop!");
			DisplayStr();
			// this code should be redundant
			//PWM_SetConfig(MOTOR_ELEVATION, PWM_CONFIG_STOPPED);	// stop both axis
			//PWM_SetConfig(MOTOR_AZIMUTH, PWM_CONFIG_STOPPED);
			ResetMotionFSM();												// prepare MotionFSM to resetart
		}
		break;

	case 'r':				// reset
		////////			Reset();													// processor reset
		//lint -fallthrough  no need for a break here, we are doing a hard reset

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		// if the Move Sequence is complete, we can redisplay the current menu
		if (IsMoveSequenceComplete() IS_TRUE)
		{
			AddDisplayStrAndNewLine("? ? ?");
			DisplayStr();
			nRetVal = MENU_SELECTED;
		}
		break;
	}

	eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
	return nRetVal;
}


// *****************************************************************************
//					S y s t e m P a r a m e t e r s M e n u ( )
// *****************************************************************************

int SystemParametersMenu(char cKeystroke)
{

	switch(cKeystroke)
	{
	case '1':				// location parameters Menu
		eCurrentMenu = LOCATION_PARAMETERS_MENU;
		break;

	case '2':				// azimuth parameters Menu
		eCurrentMenu = AZIMUTH_PARAMETERS_MENU;
		break;

	case '3':				// elevation parameters Menu
		eCurrentMenu = ELEVATION_PARAMETERS_MENU;
		break;

	case '4':				// elevation parameters Menu
		eCurrentMenu = BACKTRACKING_PARAMETERS_MENU;
		break;

	case '5':				// write parameter table to SPI flash
		if (gbSPIFlashPresent == TRUE)
		{
			WriteFlashParameterTable();		// ==> should have a return value
			AddDisplayStrAndNewLine("Table Write Complete");
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("SPI Flash Not Present");
			DisplayStr();
		}

		break;

	case '6':				// write parameter table to
		switch(InitSystemParameterTable())
		{
		case MEMORY_TIMEOUT:
			AddDisplayStrAndNewLine("SPI FLASH Memory Timeout");
			DisplayStr();
			break;

		case MEMORY_NOT_PRESENT:
			AddDisplayStrAndNewLine("SPI FLASH Not Found (ID Failure)");
			DisplayStr();
			break;

		case MEMORY_CHECKSUM_ERROR:
			AddDisplayStrAndNewLine("SPI FLASH Checksum Error");
			DisplayStr();
			break;

		case MEMORY_INITIALIZED:
			AddDisplayStrAndNewLine("SPI FLASH Initialized OK");
			DisplayStr();
			break;

		default:
			AddDisplayStrAndNewLine("SPI FLASH unknown error");
			DisplayStr();
			break;
		}
		break;

		case '7':				// elevation parameters Menu
//#ifdef USE_DS3232_RTCC
			eCurrentMenu = RTCC_DATE_TIME_MENU;
//#else
//			AddDisplayStrAndNewLine("RTCC Not Available");				// do not allow any use of menu
//			DisplayStr();
//#endif

			break;

		case 'x':				// Redisplay Main Menu
			eCurrentMenu = UTILITY_MENU;
			break;

		default:
			RuntimeError(MENU_FSM_ERROR_INVALID_STATE);
			// redisplay current menu
			break;
	}

	////	DisplayStr();										// start display (serial output) of line

	return MENU_SELECTED;

}

int LocationParametersMenu(char cKeystroke)
{
	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;
	int nStatus = 0;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// fLatitude
		AddDisplayStr("fLatitude: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fLatitude, &nStatus));
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_LATITUDE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '2':				// fLongitude
		AddDisplayStr("fLongitude: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fLongitude, &nStatus));
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_LONGITUDE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '3':				// fElevation
		AddDisplayStr("fAltitude: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAltitude, &nStatus));
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_ALTITUDE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '4':				// fRefraction
		AddDisplayStr("fRefraction: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fRefraction, &nStatus));
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_REFRACTION;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '5':				// fTimeZone
		AddDisplayStr("fTimeZone: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fTimeZone, &nStatus));
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_TIMEZONE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '6':				// ucTracking_Mode
		AddDisplayStr("ucTracking_Mode (0 = Manual, 1 = Tracking): ");
		BYTEtoHexASCIIstr(ptrRAM_SystemParameters->ucTracking_Mode, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		DisplayStr();
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_TRACKING_MODE;
		eParameterType = PARAMETER_TYPE_HEX;
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}


	////	DisplayStr();										// start display (serial output) of line

	return nRetVal;
}


int AzimuthParametersMenu(char cKeystroke)
{
	//	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;
	int nStatus = 0;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// fLatitude
		AddDisplayStr("fAZ_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_Offset, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '2':				// fAZ_SoftLimit_Reverse
		AddDisplayStr("fAZ_SoftLimit_Reverse: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_SoftLimit_Reverse, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_SOFT_LIMIT_REV;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '3':				// fAZ_SoftLimit_Forward
		AddDisplayStr("fAZ_SoftLimit_Forward: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_SoftLimit_Forward, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_SOFT_LIMIT_FWD;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '4':				// fAZ_DeadBand
		AddDisplayStr("fAZ_DeadBand: ");

		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_DeadBand, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_DEAD_BAND;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '5':				// fLatitude
		AddDisplayStr("fAZ_NightStowThreshold: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_NightStowThreshold, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_NIGHT_STOW_THRESHOLD;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '6':				// fAZ_NightStowPosition
		AddDisplayStr("fAZ_NightStowPosition: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_NightStowPosition, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_NIGHT_STOW_POS;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '7':				// fAZ_WindStowPosition
		AddDisplayStr("fAZ_WindStowPosition: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_WindStowPosition, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_WIND_STOW_POS;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();

		nRetVal = MENU_SELECTED;
		break;
	}

	DisplayStr();										// start display (serial output) of line

	return nRetVal;
}

// NOTE: Elevation menu is always available, even if there is no elevation axis, because we need to be able to set the Night Stow values
int ElevationParametersMenu(char cKeystroke)
{
	//LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;
	int nStatus = 0;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// fEL_Offset
		AddDisplayStr("fEL_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_Offset, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '2':				// fEL_SoftLimit_Reverse
		AddDisplayStr("fEL_SoftLimit_Reverse: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Reverse, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_SOFT_LIMIT_REV;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '3':				// fEL_SoftLimit_Forward
		AddDisplayStr("fEL_SoftLimit_Forward: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_SoftLimit_Forward, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_SOFT_LIMIT_FWD;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '4':				// fEL_DeadBand
		AddDisplayStr("fEL_DeadBand: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_DeadBand, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_DEAD_BAND;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '5':				// fLatitude
		AddDisplayStr("fEL_NightStowThreshold: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_NightStowThreshold, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_NIGHT_STOW_THRESHOLD;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '6':				// fEL_NightStowPosition
		AddDisplayStr("fEL_NightStowPosition: ");

		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_NightStowPosition, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_NIGHT_STOW_POS;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '7':				// fEL_WindStowPosition
		AddDisplayStr("fEL_WindStowPosition: ");

		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fEL_WindStowPosition, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_WIND_STOW_POS;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();

		nRetVal = MENU_SELECTED;
		break;
	}

	DisplayStr();										// start display (serial output) of line

	return nRetVal;

}

int BacktrackingParametersMenu(char cKeystroke)
{

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;
	int nStatus = 0;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// Backtrack option
		AddDisplayStr("Backtracking Enabled (0 = Disabled, 1 = Enabled): \t");

		BYTEtoASCIIstr(ptrRAM_SystemParameters->bBacktrackingEnabled, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_BACKTRACKING_ENABLED;
		eParameterType = PARAMETER_TYPE_HEX;
		break;

	case '2':				// Shadow start Angle
		AddDisplayStr("Panel Shadow Start Angle (degrees): ");
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fPanelShadowStartAngleDegrees, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_PANEL_SHADOW_START_ANGLE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '3':				// Shadow start Angle
		AddDisplayStr("Sun Shadow Start Angle (degrees): ");
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fSunShadowStartAngleDegrees, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SUN_SHADOW_START_ANGLE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '4':				// Shadow start Angle
		AddDisplayStr("Sun Shadow Start Height (units?): ");
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fSunShadowStartHeight, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SUN_SHADOW_START_HEIGHT;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;
	case '5':				// Single Axis Reverse softlimit
		AddDisplayStr("SinAxis Reverse SoftLimit : ");
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fSingle_SoftLimit_Reverse, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SINGLE_ANGLE_REVERSE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;

	case '6':				// Single Axis Forward softlimit
		AddDisplayStr("SinAxis Forward SoftLimit : ");
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fSingle_SoftLimit_Forward, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SINGLE_ANGLE_FORWARD;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;
	case '7':				// Single Axis Forward softlimit
		AddDisplayStr("SinAxis start Date : ");

		INT32UtoASCIIstr((BUILD_YEAR*10000+BUILD_MONTH*100+BUILD_DAY),INT32U_WIDTH,szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr((const char *)ftoa(ptrRAM_SystemParameters->fSingle_start_date, &nStatus));
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SINGLE_START_DATE;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		break;
	case '8':				// Single Axis Forward softlimit
		AddDisplayStr("SinAxis stop days : ");
		BYTEtoASCIIstr(ptrRAM_SystemParameters->fSingle_stop_days, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_SINGLE_STOP_DAYS;
		eParameterType = PARAMETER_TYPE_DECIMAL;
		break;
	case 'x':				// Redisplay Main Menu
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();

		nRetVal = MENU_SELECTED;
		break;
	}

	DisplayStr();										// start display (serial output) of line

	return nRetVal;

}



int SetRTCCDateTimeMenu(char cKeystroke)
{
	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nRetVal = COMMAND_SELECTED;
	UINT8 cI2CData;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':
		AddDisplayStr("Register 0x00 Seconds: ");

		if (ReadRTCCRegister(DS3232_REG_SECONDS, &cI2CData)	== TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");

			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_SECONDS;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");
		}
		break;

	case '2':
		AddDisplayStr("Register 0x01 Minutes: ");

		if (ReadRTCCRegister(DS3232_REG_MINUTES, &cI2CData)	== TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");
			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_MINUTES;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case '3':
		AddDisplayStr("Register 0x02 Hours: ");

		if (ReadRTCCRegister(DS3232_REG_HOURS, &cI2CData) == TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");
			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_HOURS;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case '4':
		AddDisplayStr("Register 0x03 Day: ");

		if (ReadRTCCRegister(DS3232_REG_DAY, &cI2CData)	== TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");

			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_DAY;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case '5':
		AddDisplayStr("Register 0x04 Date: ");

		if (ReadRTCCRegister(DS3232_REG_DATE, &cI2CData) == TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");
			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_DATE;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case '6':
		AddDisplayStr("Register 0x05 Month: ");

		if (ReadRTCCRegister(DS3232_REG_MONTH, &cI2CData) == TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");
			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_MONTH;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case '7':
		AddDisplayStr("Register 0x06 Year: ");

		if (ReadRTCCRegister(DS3232_REG_YEAR, &cI2CData)	== TRUE)
		{
			BYTEtoASCIIstr(cI2CData, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  _");

			nRetVal = PARAMETER_SELECTED;
			eParameter = PARAMETER_RTCC_YEAR;
			eParameterType = PARAMETER_TYPE_BCD;
		}
		else
		{
			AddDisplayStr("? ?");

		}
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = SYSTEM_PARAMETERS_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}

	DisplayStr();										// start display (serial output) of line

	return nRetVal;

}

// *****************************************************************************
//			R e a d I n p u t s M e n u ( ) 
// *****************************************************************************

int ReadInputsMenu(char cKeystroke)
{

#ifdef USE_PCA9554_IO
	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
#endif

	int nRetVal = COMMAND_SELECTED;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// motion sensor input pins
#ifdef USE_HALL_SENSOR_FEEDBACK
		AddDisplayStr("Motion (Hall) Sensor Inputs: ");
		WORDtoHexASCIIstr(GetMotionSensorState(), szfnDisplayStr);
		//WORDtoHexASCIIstr(0x00, szfnDisplayStr);
		AddDisplayStrAndNewLine(szfnDisplayStr);
#else
		AddDisplayStr("Hall Sensors Not Available");

#endif

		eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
		break;

	case '2':				// Motion Sensor tick counter
		// multi-line output display
		eResultType = RESULT_READ_INPUTS_MOTION_SENSORS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '3':				// Timers
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_TIMERS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '4':				// ADC registers
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_ADC_SETTINGS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '5':				// ADC values
		// multi-line output display
		eResultType = RESULT_READ_INPUTS_ADC;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '6':				// Switch Input values
		// multi-line output display
		eResultType = RESULT_READ_INPUTS_SWITCHES;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '7':				// Switch Input values
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_RTCC;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '8':				// Switch Input values
		// multi-line output display
		eResultType = RESULT_READ_RTCC_RAM;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case '9':				// Switch Input values
#ifdef USE_MMA8452Q_INCLINOMETER
		// multi-line output display
		eResultType = RESULT_READ_REGISTERS_INCLINOMETER;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
#else
		AddDisplayStr("Inclinometer Not Available");

#endif
		break;

	case 'a':				// Switch Input values
#ifdef USE_MMA8452Q_INCLINOMETER
		// multi-line output display
		eResultType = RESULT_READ_INCLINOMETER_DATA;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
#else
		AddDisplayStr("Inclinometer Not Available");


#endif
		break;

	case 'b':
#ifdef USE_PCA9554_IO						// enabled in config.h ONLY if PCA9554 hardware is present
		AddDisplayStr("LED State (hex): ");

		BYTEtoHexASCIIstr(GetLEDState(), szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  _0x");
		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_LED_STATE;
#else
		AddDisplayStr("LEDs Not Available");

#endif		// USE_PCA9554_IO
		DisplayStr();							// start display (serial output) of line
		break;

	case 'c':				// Switch Input values
		eResultType =	RESULT_READ_LOCATION_SETTINGS;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case 'd':				// Switch Input values
		eResultType =	RESULT_READ_BUILD_DATE;
		eResultLine = RESULT_LINE;					// first line to display is 1 of n
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}
	DisplayStr();
	return nRetVal;
}


// *****************************************************************************
//							T e s t M e n u ( )
// *****************************************************************************

int TestMenu(char cKeystroke)
{
	int nRetVal = COMMAND_SELECTED;
	char cDivideByZero;

	ClearDisplayStr();
	AddDisplayNewLine();

	switch(cKeystroke)
	{
	case '1':				// Watchdog Timer
		while(1)			// hang here to prevent WDT clear in main loop
			;
		break;

	case '2':				// Exception Handler
		cDivideByZero = cKeystroke / 0;	// force a divide by zero exception
		break;

	case 'x':				// Redisplay Main Menu
		eCurrentMenu = UTILITY_MENU;
		nRetVal = MENU_SELECTED;
		break;

	default:
		AddDisplayStrAndNewLine("? ? ?");
		DisplayStr();
		nRetVal = MENU_SELECTED;
		break;
	}

	return nRetVal;
}

// *****************************************************************************
//							D i s p l a y R e s u l t ( ) 
// *****************************************************************************

// Display of multi-line output from commands
// Displays just ONE line per pass through the function (a very few instances display two lines)
//		so multiple passes are required to display a complete data set

// returns	TRUE is a line was displayed
//			FALSE if the fgwDisplayLineCtr value goes beyond the last line
BOOL DisplayResult(void)
{

	// this may not be large enough... 
	LOCAL ARRAY char szfnDisplayStr[(DISPLAY_LINE_SIZE * 2) + 1];				// a few outputs in THIS function are actually TWO lines long
	//	WORD wADCValue;
	WORD wDutyCycle;
	INT32U lDutyCycle = 0;
	UINT8 cI2CData;
	BOOL bRetVal = TRUE;
	int nStatus;
	RTCC_DATE_TIME CurrentDateTime;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
	enum tagMotors eMotor = MOTOR_NONE;

	ClearDisplayStr();

	switch(eResultType)
	{
	case RESULT_STATUS_LAST_EL_MOVEMENT:
	case RESULT_STATUS_LAST_AZ_MOVEMENT:
	{
		if (eResultType == RESULT_STATUS_LAST_AZ_MOVEMENT)
		{
			eMotor = MOTOR_AZIMUTH;
		}
		else if (eResultType == RESULT_STATUS_LAST_EL_MOVEMENT)
		{
			eMotor = MOTOR_ELEVATION;
		}
		else
		{
			// default case
			eMotor = MOTOR_AZIMUTH;
		}

		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayStr("  Last Movement : ");

			if (eMotor == MOTOR_AZIMUTH)
			{
				AddDisplayStr("  Azimuth : ");

			}
			else if (eMotor == MOTOR_ELEVATION)
			{
				AddDisplayStr("  Elevation : ");

			}

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			switch((enum tagMoveTypes)(pgMotionStats[eMotor].eMoveType & 0x007F))
			{
			case MOVE_NONE:						// no move, stopped
				AddDisplayStr("MOVE_NONE");

				break;

			case MOVE_RUN_FWD:					// general move to right/up
				AddDisplayStr("MOVE_RUN_FWD");

				break;

			case MOVE_RUN_REV:					// general move to left/down
				AddDisplayStr("MOVE_RUN_REV");

				break;

			case MOVE_SLEW_FWD_TO_END:			// moving right/up to EOT
				AddDisplayStr("SLEW_FWD_TO_END");

				break;

			case MOVE_SLEW_REV_TO_END:			// moving left/down to EOT
				AddDisplayStr("SLEW_REV_TO_END");

				break;

#ifdef USE_MOVE_CENTER_CMD
			case MOVE_CENTER:				// move to previously determined center of travel
				AddDisplayStr("MOVE_CENTER");
				break;
#endif

			case MOVE_STALL_RECOVERY:		// short, fast, reverse move to recover from a stall
				AddDisplayStr("MOVE_STALL_RECOVERY");
				break;

			case MOVE_COMPLETE:				// not meaningful
			default:
				AddDisplayStr("Unknown");
				break;
			}

			// note: PWM direction is NOT the same as PWM config; the config includes braking, coasting, etc
			AddDisplayStr("   PWM dir: ");

			switch(pgMotionStats[eMotor].ePWMDirection)
			{
			case PWM_DIR_REVERSE:
				AddDisplayStr("REVERSE");

				break;

			case PWM_DIR_FORWARD:
				AddDisplayStr("FORWARD");

				break;

			case PWM_DIR_STOPPED:

				break;

			case PWM_DIR_UNKNOWN:
			default:
				AddDisplayStr("UNKNOWN");

				break;
			}

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

			case 2:

				AddDisplayStr("Starting Pos: ");
				INT32StoASCIIstr(pgMotionStats[eMotor].lStartingPosition, INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Ending Pos: ");
				INT32StoASCIIstr(pgMotionStats[eMotor].lEndingPosition, INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Distance: ");
				INT32StoASCIIstr(pgMotionStats[eMotor].lDistance, INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 3:
				AddDisplayStr("Accel MSI Ticks: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].wMSI_AccelerationCount, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Constant MSI Ticks: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].ulMSI_ConstantSpeedCount, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Decel MSI Ticks: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].wMSI_DecelerationCount, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 4:
				AddDisplayStr("  Min PWM MSI Ticks: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].wMSI_MinimumPWMCount, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Coast MSI Ticks: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].wMSI_CoastCount, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Total MSI Ticks: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].ulMSI_TotalCount, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 5:
				AddDisplayStr("Accel Time: ");

				INT32UtoASCIIstr((pgMotionStats[eMotor].lAccelerationEndTime - pgMotionStats[eMotor].lStartTime), INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Constant Run Time: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].lConstantSpeedEndTime - pgMotionStats[eMotor].lAccelerationEndTime, INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Decel Time: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].lDecelerationEndTime - pgMotionStats[eMotor].lConstantSpeedEndTime, INT18S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 6:
				AddDisplayStr("  Min PWM Time: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].lMinimumPWMEndTime - pgMotionStats[eMotor].lDecelerationEndTime, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Coast Time: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].lCoastEndTime - pgMotionStats[eMotor].lMinimumPWMEndTime, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Total Time: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].lCoastEndTime - pgMotionStats[eMotor].lStartTime, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;


			case 7:
				AddDisplayStr("Min Speed: ");
				INT32UtoASCIIstr((INT32U)pgMotionStats[eMotor].MinimumSpeed, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Max Speed: ");
				INT32UtoASCIIstr((INT32U)pgMotionStats[eMotor].MaximumSpeed, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
#ifdef CALC_AVERAGE_SPEED
				AddDisplayStr("  Avg Speed: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].wAverageSpeed, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
#endif

				AddDisplayStr("  Timeout: ");
				INT32UtoASCIIstr((INT32U)pgMotionStats[eMotor].LastMotionStallTimer, INT18U_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 8:
				AddDisplayStr("PWM DutyCycle: Accel Min: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bPWM_AccelDutyCycleMin, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Accel Max: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bPWM_AccelDutyCycleMax, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 9:
				AddDisplayStr("   Accel Correct Min: ");
				INT16StoASCIIstr(pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMin, INT8S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Accel Correct Max: ");
				INT16StoASCIIstr(pgMotionStats[eMotor].cPWM_AccelDutyCycleCorrectionMax, INT8S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 10:
				AddDisplayStr("PWM DutyCycle: Run Max: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bPWM_ConstantSpeedDutyCycleMax, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr(" Run Correct Min: ");
				INT16StoASCIIstr(pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMin, INT8S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr(" Run Correct Max: ");
				INT16StoASCIIstr(pgMotionStats[eMotor].cPWM_ConstantSpeedDutyCycleCorrectionMax, INT8S_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 11:
				AddDisplayStr("Motion Profile Index: Accel: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bPWMAccelMotionIndex, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Decel: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bPWMDecelMotionIndex, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Speed: ");
				WORDtoASCIIstr(pgMotionStats[eMotor].bSpeedlMotionIndex, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 12:
				AddDisplayStr("PWM DutyCycle: Decel Max: ");


				WORDtoASCIIstr(pgMotionStats[eMotor].bPWM_DecelDutyCycleMax, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Decel Min: ");					WORDtoASCIIstr(pgMotionStats[eMotor].bPWM_DecelDutyCycleMin, BYTE_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 13:
				AddDisplayStr("Accel PWM Adjust: ");

				WORDtoASCIIstr(pgMotionStats[eMotor].wAccelerationPWMAdjustmentCount, WORD_WIDTH, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Constant PWM Adjust: ");
				INT32UtoASCIIstr(pgMotionStats[eMotor].ulConstantSpeedPWMAdjustmentCount, INT18U_WIDTH, szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 14:
				AddDisplayStr("Accel Events: ");

				WORDtoHexASCIIstr(pgMotionStats[eMotor].efAccelerationMotionEvents, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Run: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efConstantSpeedMotionEvents, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Decel: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efDecelerationMotionEvents, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  MinPWM: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efMinimumPWMMotionEvents, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Coast: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efCoastingMotionEvents, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

				++fgwDisplayLineCtr;														// bump display line counter to next line
				break;

			case 15:
				AddDisplayStr("Starting Event: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efStartingEvent, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr("  Ending Event: ");
				WORDtoHexASCIIstr(pgMotionStats[eMotor].efEndingEvent, szfnDisplayStr);
				AddDisplayStr(szfnDisplayStr);

#ifdef USE_INCLINOMETER_FEEDBACK
				++fgwDisplayLineCtr;													// bump display line counter to next line
#else
				eResultLine = RESULT_LAST_LINE;											// this is the last line to display
#endif
				break;

#ifdef USE_INCLINOMETER_FEEDBACK
			case 16:
				AddDisplayStr("Start > ");

				AddDisplayStr((const char *)ftoa2(pgMotionStats[eMotor].fStartingAngle, &nStatus));
				AddDisplayStr("  Move Req > ");
				AddDisplayStr((const char *)ftoa2(pgMotionStats[eMotor].fMoveDistanceDegrees, &nStatus));
				AddDisplayStr("  End > ");
				AddDisplayStr((const char *)ftoa2(pgMotionStats[eMotor].fEndingAngle, &nStatus));
				AddDisplayStr("  Final > ");
				AddDisplayStr((const char *)ftoa2(pgMotionStats[eMotor].fFinalAngle, &nStatus));
				AddDisplayStr("  Move > ");
				AddDisplayStr((const char *)ftoa2(pgMotionStats[eMotor].fFinalAngle - pgMotionStats[eMotor].fStartingAngle, &nStatus));

				eResultLine = RESULT_LAST_LINE;												// this is the last line to display
				break;
#endif

			default:
				bRetVal = FALSE;															// nothing to display
				eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
				break;

		}
	}
	break;

	case RESULT_READ_REGISTERS_MTN_SENSOR_SETTINGS:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayStr("  Input Compare (IC) Register Values");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			AddDisplayStr("\tIC1CON IC1 Control Register:     ");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("\tIC1BUF IC1 Data FIFO       :     ");
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayNewLine();
			AddDisplayStr("\tIC2CON IC2 Control Register:     ");

			////					INT32UtoHexASCIIstr(IC2CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\tIC2BUF IC2 Data FIFO       :     ");

			////					WORDtoHexASCIIstr(IC2BUF, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayNewLine();
			AddDisplayStr("\tIC3CON IC3 Control Register:     ");


			////					INT32UtoHexASCIIstr(IC3CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("\tIC3BUF IC3 Data FIFO       :     ");

			////					WORDtoHexASCIIstr(IC3BUF, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayNewLine();
			AddDisplayStr("\tT3CON Timer 3 Control Register:  ");

			////					INT32UtoHexASCIIstr(T3CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 8:
			AddDisplayStr("\tTMR3 Timer 3 Register:           ");


			////					INT32UtoHexASCIIstr(TMR3, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 9:
			AddDisplayStr("\tPR3 Period3 Register:            ");


			////					INT32UtoHexASCIIstr(PR3, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayNewLine();
			AddDisplayStr("\tIC frequency:            ");

#ifndef _lint	// too many complex lint errors!
			////						INT32UtoASCIIstr((INT32U)(GetPeripheralClock() / (INT32U)TIMER3_CLK_PRESCALE), INT32U_WIDTH, szfnDisplayStr);
#endif	// _lint
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("Hz");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;

		}
	}
	break;


	case RESULT_READ_REGISTERS_PWM_SETTINGS:
	{
		// NOTE: The Output Compare registers used for PWM are actually 32 bits wide, but we are operating in 16 bit mode
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayStr("  Output Compare (OC) Register Values");
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			AddDisplayStr("\tOC1CON OC1 Control Register:     ");

			////					INT32UtoHexASCIIstr(OC1CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("\tOC1R OC1 Compare Register:       ");
			////					INT32UtoHexASCIIstr(OC1R, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("\tOC1RS OC1 2nd Compare Register:  ");


			////					INT32UtoHexASCIIstr(OC1RS, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\tOC2CON OC2 Control Register:     ");


			////					INT32UtoHexASCIIstr(OC2CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayStr("\tOC2R OC2 Compare Register:       ");

			////					INT32UtoHexASCIIstr(OC2R, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("\tOC2RS OC2 2nd Compare Register:  ");


			////					INT32UtoHexASCIIstr(OC2RS, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayStr("\tOC3CON OC3 Control Register:     ");

			////					INT32UtoHexASCIIstr(OC3CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 8:
			AddDisplayStr("\tOC3R OC3 Compare Register:       ");

			////					INT32UtoHexASCIIstr(OC3R, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			uart_send(dispstr);
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 9:
			AddDisplayStr("\tOC3RS OC3 2nd Compare Register:  ");


			////					INT32UtoHexASCIIstr(OC3RS, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayStr("\tOC4CON OC4 Control Register:     ");


			////					INT32UtoHexASCIIstr(OC4CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 11:
			AddDisplayStr("\tOC4R OC4 Compare Register:       ");

			////					INT32UtoHexASCIIstr(OC4R, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 12:
			AddDisplayStr("\tOC4RS OC4 2nd Compare Register:  ");


			////					INT32UtoHexASCIIstr(OC4RS, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 13:
			AddDisplayNewLine();
			AddDisplayStr("\tT2CON Timer 2 Control Register:  ");

			////					INT32UtoHexASCIIstr(T2CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 14:
			AddDisplayStr("\tTMR2 Timer 2 Register:           ");

			////					INT32UtoHexASCIIstr(TMR2, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 15:
			AddDisplayStr("\tPR2 Period2 Register:            ");

			////					INT32UtoHexASCIIstr(PR2, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 16:
			AddDisplayNewLine();
			AddDisplayStr("\tPWM frequency:           ");

#ifndef _lint	// too many complex lint errors!
			////						INT32UtoASCIIstr((INT32U)(GetPeripheralClock() / PR2), WORD_WIDTH, szfnDisplayStr);
#endif
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("Hz");


			++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;

		}
	}
	break;

	case RESULT_MOTOR_PWM_WIDTHS:
	{
		// ==>> correct display size to BYTE
		// NOTE: The Output Compare registers used for PWM are actually 32 bits wide, but we are operating in 16 bit mode
		// duty cycle is limited to 0 to 100, so a BYTE is large enough
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayStr("\tCurrent PWM1 Duty Cycle: ");


			////					wDutyCycle = (INT16U)(OC1R / (PR2/100L));
			WORDtoASCIIstr(wDutyCycle, BYTE_WIDTH, szfnDisplayStr);						// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			AddDisplayStr("\tCurrent PWM2 Duty Cycle: ");


			////					lDutyCycle = (INT32U)OC2R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);


			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("\tCurrent PWM3 Duty Cycle: ");

			////					lDutyCycle = (INT32U)OC3R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("\tCurrent PWM4 Duty Cycle: ");

			////					lDutyCycle = (INT32U)OC4R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\tPWM frequency:           ");

#ifndef _lint	// too many complex lint errors!
			////						INT32UtoASCIIstr((INT32U)(GetPeripheralClock() / PR2), WORD_WIDTH, szfnDisplayStr);
#endif
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("Hz");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	// is there a way to read the configuration bits registers?
	// actual physical address is 0xf80004 in PROGRAM SPACE (determined from MPLAP map file)
	// AddDisplayStr("\tFBORPOR: BOR && POR Device Configuration Register: ");
	// WORDtoHexASCIIstr(FBORPOR, szfnDisplayStr);
	AddDisplayStr(szfnDisplayStr);

	case RESULT_READ_INPUTS_MOTION_SENSORS:
	{
		// this directly reads the register values
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStrAndNewLine("  Motion (Hall) Sensor Tracking Values");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			AddDisplayStrAndNewLine("\tMechanical Orientation: ");

			CurrentMechanicalOrientation_Format(szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("\tTotal Counts: Az ");

			WORDtoASCIIstr((WORD)pgulMoveTotalMSICtr[AXIS_AZIMUTH], WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("  El ");

			WORDtoASCIIstr((WORD)pgulMoveTotalMSICtr[AXIS_ELEVATION], WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_REGISTERS_TIMERS:
	{
		// the timers are read through the Microchip I/O library, NOT neccesarily directly from the hardware
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStr("  Timer Register Values");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			AddDisplayStr("\tCore Timer(Scheduler 5mS Tick Timing): \t");

			////					WORDtoASCIIstr(ReadCoreTimer(), WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayNewLine();
			AddDisplayStr("\tT1CON Timer 1 Control Register:  ");

			////					INT32UtoHexASCIIstr(T1CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("\tTMR1 Timer 1 Register:           ");


			////					INT32UtoHexASCIIstr(TMR1, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\tPR1 Period1 Register:            ");

			////					INT32UtoHexASCIIstr(PR1, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayNewLine();
			AddDisplayStr("\tT2CON Timer 2 Control Register:  ");


			////					INT32UtoHexASCIIstr(T2CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("\tTMR2 Timer 2 Register:           ");

			////					INT32UtoHexASCIIstr(TMR2, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayStr("\tPR2 Period2 Register (PWM):      ");

			////					INT32UtoHexASCIIstr(PR2, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 8:
			AddDisplayNewLine();
			AddDisplayStr("\tT3CON Timer 3 Control Register:  ");

			////					INT32UtoHexASCIIstr(T3CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 9:
			AddDisplayStr("\tTMR3 Timer 3 Register:           ");


			////					INT32UtoHexASCIIstr(TMR3, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayStr("\tPR3 Period3 Register (Encoders): ");

			////					INT32UtoHexASCIIstr(PR3, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			//					break;

			// TBD: timers 4, 5

			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;


		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_REGISTERS_ADC_SETTINGS:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStrAndNewLine("  ADC Register values");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;
#ifdef NOTDEF
		case 1:
			AddDisplayStr("\tADCON0: Channel Select, Status, ADON: \t");

			WORDtoHexASCIIstr(ADCON0, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			++fgwDisplayLineCtr;														// bump display line counter to next line

			break;

		case 2:
			AddDisplayStr("\tADCON1: Vref, Port Config: \t\t");

			WORDtoHexASCIIstr(ADCON1, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("\tADCON2: Acq time, Clock Select: \t");

			WORDtoHexASCIIstr(ADCON2, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\tTRISA: Port A Config: \t\t\t");


			WORDtoHexASCIIstr(TRISA, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;


		case 5:
			AddDisplayStr("\tCCP2CON: CCP Special Event Trigger: \t");
			WORDtoHexASCIIstr(CCP2CON, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("\tADRESx: A/D Result Registers: \t\t");
			WORDtoHexASCIIstr((ADRESH * 256) + ADRESL, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;
#endif

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;


	case RESULT_READ_INPUTS_ADC:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStrAndNewLine("  ADC Register values");
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;
#ifdef NOTDEF
		case 1:
			AddDisplayStr("Battery (AN0): \t\t");


			wADCValue = ADC_Read_Basic(ADC_CH_BATTERY);
			WORDtoASCIIstr(wADCValue, TEN_BIT_WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("Thermistor (AN1/AN2): \t");

			wADCValue = ADC_Read_Basic(ADC_CH_THERMISTOR);
			WORDtoASCIIstr(wADCValue, TEN_BIT_WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("End-of-Travel OUT (AN4): ");


			wADCValue = ADC_Read_Basic(ADC_CH_EOT_OUT);
			WORDtoASCIIstr(wADCValue, TEN_BIT_WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;
#endif

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_INPUTS_SWITCHES:
	{
#ifdef USE_PCA9554_IO												// enable in config.h ONLY if PCA9554 hardware is present
		// display current state of input switches, debounced switch events
		AddDisplayStr("Raw Input Pins: ");


		BYTEtoHexASCIIstr(GetInputSwitchState(), szfnDisplayStr);		// this actually reads the hardware
		AddDisplayStr(szfnDisplayStr);

		AddDisplayStr("\tDebounced Events: ");
		WORDtoHexASCIIstr(efSwitchEvents, szfnDisplayStr);				// event flags are 16 bit words
		AddDisplayStr(szfnDisplayStr);
		eStreamLine = STREAM_LINE;					// mark display of {some} line of data
#else
		AddDisplayStr("\tInput Switches Not Available");

#endif

		bRetVal = FALSE;															// nothing to display
		eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
	}
	break;

	case RESULT_READ_REGISTERS_RTCC:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
//#ifdef USE_DS3232_RTCC
			AddDisplayStrAndNewLine("  DS3232 RTCC Register values");
			AddDisplayNewLine();


			++fgwDisplayLineCtr;														// bump display line counter to next line
//#else
//			AddDisplayStrAndNewLine("  DS3232 RTCC Not Available");
//
//
//			bRetVal = FALSE;															// nothing to display
//			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
//#endif
			break;

		case 1:
			AddDisplayStr("Register 0x00 Seconds:\t\t");

			if (ReadRTCCRegister(DS3232_REG_SECONDS, &cI2CData)	== TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("Register 0x01 Minutes:\t\t");

			if (ReadRTCCRegister(DS3232_REG_MINUTES, &cI2CData)	== TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("Register 0x02 Hours :\t\t");

			if (ReadRTCCRegister(DS3232_REG_HOURS, &cI2CData) == TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("\n Register 0x03 Day:\t\t");

			if (ReadRTCCRegister(DS3232_REG_DAY, &cI2CData)	== TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr(pstrDayText[BCDtoBYTE(cI2CData)]);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayStr("\nRegister 0x04 Date:\t\t");

			if (ReadRTCCRegister(DS3232_REG_DATE, &cI2CData) == TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("Register 0x05 Month:\t\t");

			if (ReadRTCCRegister(DS3232_REG_MONTH, &cI2CData) == TRUE)
			{
				BYTEtoASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);
				AddDisplayStr(pstrMonthText[BCDtoBYTE(cI2CData)]);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayStr("\nRegister 0x06 Year:\t\t");

			if (ReadRTCCRegister(DS3232_REG_YEAR, &cI2CData) == TRUE)
			{
				int year = 2000;
				year+=(int)cI2CData;

				//INT32StoASCIIstr(year,4, szfnDisplayStr);					// this actually reads the hardware
				//itoa(year,szfnDisplayStr,10);
				sprintf(szfnDisplayStr,"%d",year);
				AddDisplayStr(szfnDisplayStr);
				AddDisplayNewLine();
			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;
#ifdef USE_DS3232_RTCC
		case 8:
			AddDisplayStr("Register 0x07 Alarm1 Seconds:\t");

			if (ReadRTCCRegister(DS3232_REG_ALARM1, &cI2CData) == TRUE)
			{
				BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);

			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 9:
			AddDisplayStr("Register 0x0E Control Register:\t");

			if (ReadRTCCRegister(DS3232_REG_CR, &cI2CData) == TRUE)
			{
				BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);

			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayStr("Register 0x0F Status Register:\t");

			if (ReadRTCCRegister(DS3232_REG_SR, &cI2CData) == TRUE)
			{
				BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);

			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			//++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
#endif


		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_RTCC_RAM:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
#ifdef USE_DS3232_RTCC
			AddDisplayStrAndNewLine("  DS3232 RTCC RAM, MCU RAM Copies");

			if (ReadRTCCRAMParameterTables() IS_FALSE)									// read current RTCC RAM values into MCU RAM
			{
				AddDisplayStrAndNewLine("  Unable to read RTCC RAM");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
#else
			AddDisplayStrAndNewLine("  DS3232 RTCC Not Available");


			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
#endif
			break;

		case 1:
			AddDisplayStr("Checksum:\t\t\t\t0x");


			WORDtoHexASCIIstr(ptrRTCC_RAM_AppParameters->unChecksum, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("Magic Number:\t\t\t\t0x");


			WORDtoHexASCIIstr(ptrRTCC_RAM_AppParameters->unMagic, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("Azimuth at SPA Calc (ticks):  \t");


			INT32StoASCIIstr(ptrRTCC_RAM_AppParameters->lAzimuthAtSPA, INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("Elevation at SPA Calc (ticks):\t");

			INT32StoASCIIstr(ptrRTCC_RAM_AppParameters->lAzimuthAtSPA, INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayStr("Last SPA Calc Azimuth (degrees):\t");


			AddDisplayStr((const char *)ftoa(ptrRTCC_RAM_AppParameters->fSPACalculation_AZ, &nStatus));

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("Last SPA Calc Elevation (degrees):\t");

			AddDisplayStr((const char *)ftoa(ptrRTCC_RAM_AppParameters->fSPACalculation_EL, &nStatus));
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayStr("Last SetPoint Azimuth (degrees):\t");

			AddDisplayStr((const char *)ftoa(ptrRTCC_RAM_AppParameters->fSetPoint_AZ, &nStatus));
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 8:
			AddDisplayStr("Last SetPoint Elevation (degrees):\t");


			AddDisplayStr((const char *)ftoa(ptrRTCC_RAM_AppParameters->fSetPoint_EL, &nStatus));
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 9:
			AddDisplayStr("Update Counter:\t\t\t\t0x");

			WORDtoHexASCIIstr(ptrRTCC_RAM_AppParameters->unUpdateCtr, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayStr("End Marker:\t\t\t\t0x");


			WORDtoHexASCIIstr(ptrRTCC_RAM_AppParameters->unEndMarker, szfnDisplayStr);
			AddDisplayStrAndNewLine(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

			// these are from the separate Mechanical Orientation structure (MSI Tick Counter values)
		case 11:
			AddDisplayStr("Mech Orientation Last Azimuth (ticks):\t  ");

			INT32StoASCIIstr(ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth, INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 12:
			AddDisplayStr("Mech Orientation Last Elevation (ticks):");

			INT32StoASCIIstr(ptrRTCC_RAM_MechanicalOrientation->lLastElevation, INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_REGISTERS_INCLINOMETER:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
#ifdef USE_MMA8452Q_INCLINOMETER
			AddDisplayStrAndNewLine("  MMA8452Q Inclinometer Registers");

			if (ReadRTCCRAMParameterTables() IS_FALSE)									// read current RTCC RAM values into MCU RAM
			{
				AddDisplayStrAndNewLine("  Unable to read  Inclinometer Registers");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
#else
			AddDisplayStrAndNewLine("  MMA8452Q Inclinometer Available");
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
#endif
			break;

		case 1:
			AddDisplayStr("Register 0x00 Status:\t\t\t");
			if (gIsMma845Enabled)
			{
				if (ReadInclinometerRegister(STATUS_00_REG, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_STATUS, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);
				}
				else
				{
					AddDisplayStr("? ? ?");

				}

			}

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("Register 0x01 Out X MSB:\t\t");

			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_X_MSB_REG, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_X_H, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("Register 0x02 Out X LSB:\t\t");
			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_X_LSB_REG, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_X_L, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 4:
			AddDisplayStr("Register 0x03 Out Y MSB:\t\t");

			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_Y_MSB_REG, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Y_H, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 5:
			AddDisplayStr("Register 0x04 Out Y LSB:\t\t");
			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_Y_LSB_REG, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Y_L, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 6:
			AddDisplayStr("Register 0x05 Out Z MSB:\t\t");
			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_Z_MSB_REG, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Z_H, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 7:
			AddDisplayStr("Register 0x06 Out Z LSB:\t\t");
			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_Z_LSB_REG, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Z_L, &cI2CData)	== TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 8:
			AddDisplayStr("Register 0x0B System Mode:\t\t");

			if (ReadInclinometerRegister(SYSMOD_REG, &cI2CData) == TRUE)
			{
				BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);

			}
			else
			{
				AddDisplayStr("? ? ?");
				memset(dispstr,0,sizeof(dispstr));
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;


		case 9:
			AddDisplayStr("Register 0x0C Interrupt Source:\t\t");

			if (ReadInclinometerRegister(INT_SOURCE_REG, &cI2CData) == TRUE)
			{
				BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
				AddDisplayStr(szfnDisplayStr);


			}
			else
			{
				AddDisplayStr("? ? ?");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 10:
			AddDisplayStr("Register 0x0D Who Am I:\t\t\t");
			if (gIsMma845Enabled)
			{
				if (ReadInclinometerRegister(WHO_AM_I_REG, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_WHO_AM_I, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 11:
			AddDisplayStr("Register 0x2A Control Reg 1:\t\t");
			if (gIsMma845Enabled)
			{
				if (ReadInclinometerRegister(CTRL_REG1, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_CTRL1, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 12:
			AddDisplayStr("Register 0x2B Control Reg 2:\t\t");
			if (gIsMma845Enabled)
			{
				if (ReadInclinometerRegister(CTRL_REG2, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_CTRL2, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 13:
			AddDisplayStr("Register 0x2C Control Reg 3:\t\t");
			if (gIsMma845Enabled)
			{
				if (ReadInclinometerRegister(CTRL_REG3, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_CTRL3, &cI2CData) == TRUE)
				{
					BYTEtoHexASCIIstr(cI2CData, szfnDisplayStr);					// this actually reads the hardware
					AddDisplayStr(szfnDisplayStr);

				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			//++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;



		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_READ_INCLINOMETER_DATA:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
#ifdef USE_MMA8452Q_INCLINOMETER
			AddDisplayStrAndNewLine("  MMA8452Q Inclinometer Angle Data");

			if (ReadRTCCRAMParameterTables() IS_FALSE)									// read current RTCC RAM values into MCU RAM
			{
				AddDisplayStrAndNewLine("  Unable to read Inclinometer Data");

			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
#else
			AddDisplayStrAndNewLine("  MMA8452Q InclinometerC Not Available");

			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
#endif
			break;

		case 1:
			AddDisplayStr("Registers 0x01, 0x02 Out X:\t\t");

			if (gIsMma845Enabled)
			{
				// read 8MSB of X axis
				IGNORE_RETURN_VALUE MMA845x_Active();
				if (ReadInclinometerRegister(OUT_X_MSB_REG, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 16);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(OUT_X_LSB_REG, &cI2CData)	== TRUE)
					{
						sResult += (cI2CData / 16);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_X_H, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 256);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(LIS2HH12_OUT_X_L, &cI2CData)	== TRUE)
					{
						sResult += (INT16)(cI2CData);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			AddDisplayStr("Registers 0x03, 0x04 Out Y:\t\t");

			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				// read 8MSB of X axis
				if (ReadInclinometerRegister(OUT_Y_MSB_REG, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 16);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(OUT_Y_LSB_REG, &cI2CData)	== TRUE)
					{
						sResult += (cI2CData / 16);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Y_H, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 256);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(LIS2HH12_OUT_Y_L, &cI2CData)	== TRUE)
					{
						sResult += (INT16)(cI2CData);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 3:
			AddDisplayStr("Registers 0x05, 0x06 Out Z:\t\t");
			if (gIsMma845Enabled)
			{
				IGNORE_RETURN_VALUE MMA845x_Active();
				// read 8MSB of X axis
				if (ReadInclinometerRegister(OUT_Z_MSB_REG, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 16);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(OUT_Z_LSB_REG, &cI2CData)	== TRUE)
					{
						sResult += (cI2CData / 16);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			else
			{
				if (ReadInclinometerRegister(LIS2HH12_OUT_Z_H, &cI2CData)	== TRUE)
				{
					INT16 sResult = (INT16)(cI2CData * 256);				// shift 8MSB to correct position in INT16 result
					BOOL bNegative = FALSE;
					// check for negative value
					if ((cI2CData & OUT_SIGN_MASK) == OUT_SIGN_MASK)	// check for negative number
					{
						// value is negative, so keep track of negative state
						bNegative = TRUE;
					}
					// read 4LSB of x axis; data is in the MSB position
					if (ReadInclinometerRegister(LIS2HH12_OUT_Z_L, &cI2CData)	== TRUE)
					{
						sResult += (INT16)(cI2CData);						// shift 4LSB to LSB position
						if (bNegative == TRUE)							// check for 8MSB negative number
						{
							sResult |= (INT16)VALUE_SIGN_BIT;			// restore sign bit in MSB location, also sets 3 unused bits
						}
						INT16StoASCIIstr(sResult, INT16S_WIDTH, szfnDisplayStr);
						AddDisplayStr(szfnDisplayStr);

					}
				}
				else
				{
					AddDisplayStr("? ? ?");

				}
			}
			++fgwDisplayLineCtr;														// bump display line counter to next line
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;


		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;
	case      RESULT_READ_BUILD_DATE:
	{

		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStrAndNewLine("\nBuild Date:");
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			strcpy(szfnDisplayStr, "\tBuild Date: ");
			AddDisplayStr(szfnDisplayStr);


			ADDBUILDDateTime(szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			// bump display line counter to next line
			break;

		case 2:
			strcpy(szfnDisplayStr, "\tRTCC Date: ");
			memset(dispstr,0,sizeof(dispstr));


			AddDisplayStr(szfnDisplayStr);

			if (ReadRTCCDateTime(ptrDateTime) != TRUE)
			{
				strcat(szfnDisplayStr, " ERR\n\r");

			}
			else
				IGNORE_RETURN_VALUE ADD2RTCCDateTime(szfnDisplayStr, ptrDateTime);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;
			break;

		case 3:
			strcpy(szfnDisplayStr, "\tDiff: ");

			if (ReadRTCCDateTime(ptrDateTime) != TRUE)
			{
				strcat(szfnDisplayStr, " ERR\n\r");

			}
			else
				GETDIFF(szfnDisplayStr, ptrDateTime);
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;
		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}

	}
	break;
	case RESULT_READ_LOCATION_SETTINGS:
	{
		switch(fgwDisplayLineCtr)
		{
		case 0:
			AddDisplayNewLine();
			AddDisplayStrAndNewLine("\nLocation Settings:");

			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 1:
			strcpy(szfnDisplayStr, "\tLatitude: ");

			strcat(szfnDisplayStr, ftoa2(ptrRAM_SystemParameters->fLatitude, &nStatus));

			AddDisplayStr(szfnDisplayStr);
			++fgwDisplayLineCtr;														// bump display line counter to next line
			break;

		case 2:
			strcpy(szfnDisplayStr, "\tLongitude: ");

			strcat(szfnDisplayStr, ftoa2(ptrRAM_SystemParameters->fLongitude, &nStatus));
			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;
			break;

		case 3:
			strcpy(szfnDisplayStr, "\tAltitude: ");

			strcat(szfnDisplayStr, ftoa2(ptrRAM_SystemParameters->fAltitude, &nStatus));

			AddDisplayStr(szfnDisplayStr);

			++fgwDisplayLineCtr;
			break;

		case 4:
			strcpy(szfnDisplayStr,"\tTimeZone: ");
			strcat(szfnDisplayStr, ftoa2(ptrRAM_SystemParameters->fTimeZone, &nStatus));
			AddDisplayStr(szfnDisplayStr);
			++fgwDisplayLineCtr;
			eResultLine = RESULT_LAST_LINE;												// this is the last line to display
			break;

		default:
			bRetVal = FALSE;															// nothing to display
			eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
			break;
		}
	}
	break;

	case RESULT_NONE:
	default:
		bRetVal = FALSE;															// nothing to display
		eResultLine = RESULT_LAST_LINE;												// this is (effectively) the last line to display
		break;
	}

	if (eResultLine != RESULT_LAST_LINE)
		AddDisplayNewLine();							// add line terminator unless this is the LAST line

	DisplayStr();										// start display (serial output) of line

	return bRetVal;										// TRUE if more data to display, FALSE if last line
}


// *****************************************************************************
//							D i s p l a y S t a t u s ( ) 
// *****************************************************************************

// streaming status of current motor direction, speed, position
// this operates in two modes:
//		on each call, displays most recent value(s) (keep in mind this may be oversampled or undersampled, depending on the rate of change of the data being displayed)
//			runs until keystroke
//		on each call, displays the next data from a list (specifically runtime errors, motion errors, or SPA move tracking)
//			runs until keystroke or end of list
BOOL DisplayStatus(void)
{

	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	//	unsigned int nADCValue;
	INT32U lDutyCycle;
	BOOL bRetVal;
	int eMotor = 0;			// kludge!

	ClearDisplayStr();

	switch(eCurrentStatus)
	{
#ifdef USE_AZIMUTH
	case AZ_FSM_STATUS:
		// display speed and position
		eMotor = MOTOR_AZIMUTH;
		AddDisplayStr("AZ Spd: ");
		INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr(" Pos: ");
		INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);

		// display FSM state strings
		AddDisplayStr(" Cmd: ");
		AddDisplayStr(GetMotionPhaseStateString(eMotor));
		AddDisplayStr(" Mtn: ");
		AddDisplayStr(GetMotionStateString(eMotor));
		AddDisplayStr(" MtnSeq: ");
		AddDisplayStr(GetMoveSeqStateString());
		AddDisplayStr(" BtnPrc: ");

		//	AddDisplayStr(GetButtonProcessingStateString());

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data

		bRetVal = TRUE;								// always returns true; no limit to available data
		break;
#endif	//  USE_AZIMUTH

#ifdef USE_ELEVATION
	case EL_FSM_STATUS:
		// display speed and position
		eMotor = MOTOR_ELEVATION;

		AddDisplayStr("\tEL Spd: ");
		INT32UtoASCIIstr(CurrentSpeed_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		AddDisplayStr("  Pos: ");
		INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);

		// display FSM state strings
		AddDisplayStr(" Cmd: ");
		AddDisplayStr(GetMotionPhaseStateString(eMotor));
		AddDisplayStr("   Motion: ");
		AddDisplayStr(GetMotionStateString(eMotor));
		AddDisplayStr("   Motion Sequencer: ");
		AddDisplayStr(GetMotionSeqStateString());

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data

		bRetVal = TRUE;								// always returns true; no limit to available data
		break;
#endif	// USE_ELEVATION


#ifdef MOTION_ERROR_TABLE
		// ==>> only applies to USE_PWM_DUTY_CYCLE
	case MOTION_ERROR_STATUS:
	{
		// display the contents of the Acceleration Speed/Motion Error buffer
		// note that we do not actually remove anything from the buffer; we just bump a display index that is initialized to 0
		if ((fgwMotionErrorDisplayIndex <= pgwMotionProfileSpeedErrorIndex) && (fgwMotionErrorDisplayIndex < SPEED_ERROR_TABLE_LEN))
		{
			AddDisplayStr("MP Inx: ");
			BYTEtoASCIIstr(pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].bMotionProfileIndex, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("\tExp Spd: ");
			INT32UtoASCIIstr((INT32U)pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].ExpectedSpeed, INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("\tMeas Spd: ");
			INT32UtoASCIIstr((INT32U)pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].MeasuredSpeed, INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("\tSpd Err: ");
			INT32StoASCIIstr((INT32S)pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].SpeedError, INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("\tPWM Corr: ");
			INT16StoASCIIstr(pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].cPWMCorrection, INT8S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("\tPWM: ");
			BYTEtoASCIIstr(pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].bPWM, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#ifdef USE_INCLINOMETER_FEEDBACK
			AddDisplayStr("\tLast Move: ");


			INT32UtoASCIIstr((INT32U)pgMotionProfileSpeedErrorTbl[fgwMotionErrorDisplayIndex].LastMoveDistanceTicks, INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#endif

			eStreamLine = STREAM_LINE;				// mark display of {some} line of data

			++fgwMotionErrorDisplayIndex;			// bump display index
			bRetVal = TRUE;							// data to display
		}
		else
		{
			eStreamLine = STREAM_LAST_LINE;			// mark display of LAST line of available data
			AddDisplayStr(" -end-");				// really not a good way to do things; does not wait for completion

			ClearMotionErrorTable();				// clear the Motion Error buffer

			bRetVal = FALSE;						// no more data to display
		}
	}
	break;
#endif	// MOTION_ERROR_TABLE

	case MOTOR_STATUS:
	{
		// DEBUG_PIN1_HI;

#ifdef USE_AZIMUTH
		eMotor = MOTOR_AZIMUTH;
		AddDisplayStr("AZ ");
		switch(pgePWMDirection[eMotor])
		{
		case PWM_DIR_REVERSE:
			AddDisplayStr("R S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#ifdef CALC_AVERAGE_SPEED
			AddDisplayStr(" Sa: ");

			WORDtoASCIIstr(pgwAverageSpeed, WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Sae: ");
			INT16StoASCIIstr(pgnErrorFromAverage, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Se: ");
			INT16StoASCIIstr(pgnErrorFromSample, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
#endif

			AddDisplayStr(" S Err: ");
			INT32StoASCIIstr((INT32S)pgsSpeedError[eMotor], INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" PWM Corr: ");
			INT16StoASCIIstr(pgcDutyCycleCorrection[eMotor], INT8S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("  PWM1 DC: ");
			////						lDutyCycle = (INT32U)OC1R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			AddDisplayStr("  PWM2 DC: ");
			////						lDutyCycle = (INT32U)OC2R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			break;

		case PWM_DIR_STOPPED:
			AddDisplayStr("S  S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			break;

		case PWM_DIR_FORWARD:
			AddDisplayStr("F S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#ifdef CALC_AVERAGE_SPEED
			AddDisplayStr(" Sa: ");

			WORDtoASCIIstr(pgwAverageSpeed, WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Sae: ");
			INT16StoASCIIstr(pgnErrorFromAverage, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Se: ");
			INT16StoASCIIstr(pgnErrorFromSample, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
#endif

			AddDisplayStr(" S Err: ");
			INT32StoASCIIstr((INT32S)pgsSpeedError[eMotor], INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" PWM Corr: ");
			INT16StoASCIIstr(pgcDutyCycleCorrection[eMotor], INT8S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("  PWM1 DC: ");
			////						lDutyCycle = (INT32U)OC1R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			AddDisplayStr("  PWM2 DC: ");
			////						lDutyCycle = (INT32U)OC2R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			break;
		case PWM_DIR_UNKNOWN:

		default:
			AddDisplayStr("?  ");

			break;
		}
#endif	// USE_AZIMUTH

#ifdef USE_ELEVATION
		eMotor = MOTOR_ELEVATION;
		AddDisplayStr(" EL ");

		switch(pgePWMDirection[eMotor])
		{
		case PWM_DIR_REVERSE:
			AddDisplayStr("R S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#ifdef CALC_AVERAGE_SPEED
			AddDisplayStr(" Sa: ");
			WORDtoASCIIstr(pgwAverageSpeed, WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Sae: ");
			INT16StoASCIIstr(pgnErrorFromAverage, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Se: ");
			INT16StoASCIIstr(pgnErrorFromSample, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
#endif

			AddDisplayStr(" S Err: ");
			INT32StoASCIIstr((INT32S)pgsSpeedError[eMotor], INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" PWM Corr: ");
			INT16StoASCIIstr(pgcDutyCycleCorrection[eMotor], INT8S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("  PWM3 DC: ");
			lDutyCycle = (INT32U)OC3R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");
			AddDisplayStr("  PWM4 DC: ");
			lDutyCycle = (INT32U)OC4R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			break;

		case PWM_DIR_STOPPED:
			AddDisplayStr("S  S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("  P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			break;

		case PWM_DIR_FORWARD:
			AddDisplayStr("F S: ");

			INT32UtoASCIIstr((INT32U)CurrentSpeed_Read(eMotor), INT18U_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

#ifdef CALC_AVERAGE_SPEED

			WORDtoASCIIstr(pgwAverageSpeed, WORD_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Sae: ");
			INT16StoASCIIstr(pgnErrorFromAverage, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" Se: ");
			INT16StoASCIIstr(pgnErrorFromSample, INT16S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);
#endif

			AddDisplayStr(" S Err: ");
			INT32StoASCIIstr((INT32S)pgsSpeedError[eMotor], INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" PWM Corr: ");
			INT16StoASCIIstr(pgcDutyCycleCorrection[eMotor], INT8S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr(" P: ");
			INT32StoASCIIstr(CurrentPosition_Read(eMotor), INT18S_WIDTH, szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			AddDisplayStr("  PWM3 DC: ");
			lDutyCycle = (INT32U)OC3R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			AddDisplayStr("  PWM4 DC: ");
			lDutyCycle = (INT32U)OC4R / (INT32U)(PR2/100);
			WORDtoASCIIstr((INT16U)lDutyCycle, BYTE_WIDTH, szfnDisplayStr);				// note that we can limit the width to just 3 characters
			AddDisplayStr(szfnDisplayStr);
			AddDisplayStr("%");

			break;
		case PWM_DIR_UNKNOWN:

		default:
			AddDisplayStr("?  ");
			break;
		}
#endif	// USE_ELEVATION


		// DEBUG_PIN1_LOW;
		eStreamLine = STREAM_LINE;					// mark display of {some} line of data

		bRetVal = TRUE;								// always returns true; no limit to available data
	}
	break;


	case INPUT_SWITCH_STATUS:
	{
#ifdef USE_PCA9554_IO							// enable in config.h ONLY if PCA9554 hardware is present
		// display current state of input switches, debounced switch events
		AddDisplayStr("Raw Input Pins: ");

		BYTEtoHexASCIIstr(GetInputSwitchState(), szfnDisplayStr);		// this actually reads the hardware
		AddDisplayStr(szfnDisplayStr);

		AddDisplayStr("\tDebounced Events: ");
		WORDtoHexASCIIstr(efSwitchEvents, szfnDisplayStr);				// event flags are 16 bit words
		AddDisplayStr(szfnDisplayStr);

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data

		bRetVal = TRUE;								// always returns true; no limit to available data
#else
		AddDisplayStr("\tInput Switches Not Available");

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data
		bRetVal = FALSE;								// always returns true; no limit to available data
#endif
	}
	break;


	case INCLINOMETER_STATUS:
	{
#ifdef USE_MMA8452Q_INCLINOMETER
		INCLINOMETER_SAMPLE Inclination;

		// read and display accelerometer values
		if (ReadInclinometerSample(&Inclination) == TRUE)
		{
			IGNORE_RETURN_VALUE FormatInclination(szfnDisplayStr, &Inclination);
			AddDisplayStr(szfnDisplayStr);
			eStreamLine = STREAM_LINE;					// mark display of {some} line of data
			bRetVal = TRUE;								// always returns true; no limit to available data
		}
		else
		{
			AddDisplayStr("\tInclinometer Read Error");

			eStreamLine = STREAM_LINE;					// mark display of {some} line of data
			bRetVal = FALSE;							// cound not read, nothing more to display
		}
#else
		AddDisplayStr("\tInclinometer Not Available");

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data
		bRetVal = FALSE;								// always returns true; no limit to available data
#endif		// USE_MMA8452Q_INCLINOMETER
	}
	break;

	case INCLINOMETER_AVERAGE_STATUS:
	{
#ifdef USE_MMA8452Q_INCLINOMETER
		INCLINOMETER_SAMPLE Inclination;

		// read and display accelerometer values
		if (ReadInclinometerSample(&Inclination) == TRUE)
		{
			IGNORE_RETURN_VALUE AverageInclinometerSample(&Inclination);
			IGNORE_RETURN_VALUE FormatAverageInclination(szfnDisplayStr);

			AddDisplayStr(szfnDisplayStr);

			eStreamLine = STREAM_LINE;					// mark display of {some} line of data
			bRetVal = TRUE;								// always returns true; no limit to available data
		}
		else
		{
			AddDisplayStr("\tInclinometer Read Error");

			eStreamLine = STREAM_LINE;					// mark display of {some} line of data
			bRetVal = FALSE;							// cound not read, nothing more to display
		}
#else
		AddDisplayStr("\tInclinometer Not Available");

		eStreamLine = STREAM_LINE;					// mark display of {some} line of data
		bRetVal = FALSE;								// always returns true; no limit to available data
#endif		// USE_MMA8452Q_INCLINOMETER
	}
	break;

	case POLAR_AXIS_MOVE_STATUS:
#if defined(USE_SINGLE_POLAR_AXIS) && defined(USE_POLAR_AXIS_MOVE_TABLE)
	{
		int nStatus = 0;
		// display the contents of the runtime error buffer
		// note that we do not actually remove anything from the buffer; we just bump a display index that is initialized to 0
		if ((fgbPolarAxisMoveDisplayIndex < POLAR_AXIS_MOVE_TBL_LEN) && (pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].fAzimuthAngleDegrees != 0.0))
		{
			eStreamLine = STREAM_LINE;				// mark display of {some} line of data
			memset(dispstr,0,sizeof(dispstr));
			strcpy(szfnDisplayStr, "hr: ");	// start string buffer

			BYTEtoASCIIstr(pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].cHours, szfnDisplayStr + strlen(szfnDisplayStr));
			strcat(szfnDisplayStr, "  Az: ");
			strcat(szfnDisplayStr, (const char *)ftoa2(pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].fAzimuthAngleDegrees, (int *)&nStatus));
			strcat(szfnDisplayStr, "  MTlt: ");
			strcat(szfnDisplayStr, (const char *)ftoa2(pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].fModuleTiltAngleDegrees, (int *)&nStatus));
			strcat(szfnDisplayStr, "  Bcktrk: ");
			strcat(szfnDisplayStr, (const char *)ftoa2(pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].fBackTrackAngleDegrees, (int *)&nStatus));
			strcat(szfnDisplayStr, "  S Elev: ");
			strcat(szfnDisplayStr, (const char *)ftoa2(pgPolarAxisMoveTbl[fgbPolarAxisMoveDisplayIndex].fSunElevationAngleDegrees, (int *)&nStatus));
			strcat(szfnDisplayStr, "  S Az: ");

			AddDisplayStr(szfnDisplayStr);

			++fgbPolarAxisMoveDisplayIndex;			// bump display index
			bRetVal = TRUE;							// data to display
		}
		else
		{
			eStreamLine = STREAM_LAST_LINE;			// mark display of LAST line of available data
			AddDisplayStr(" -end-");				// really not a good way to do things; does not wait for completion

			bRetVal = FALSE;						// no more data to display
		}
	}
#else
	AddDisplayStr("\tNot Available");

	eStreamLine = STREAM_LINE;					// mark display of {some} line of data
	bRetVal = FALSE;							// always returns true; no limit to available data
#endif

	break;

	case SUN_ANGLES_STATUS:
	{
		RTCC_DATE_TIME CurrentDateTime;
		PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
		PRIVATE SmartTrakOrientation SPAOrientation;
		//  int nStatus = 0;
		eStreamLine = STREAM_LINE;				// mark display of {some} line of data

		switch(fgbStreamSunAnglesState)
		{
		case 0:
			// read, format, and display date-time
			if (ReadRTCCDateTime(ptrDateTime) != TRUE)
			{
				AddDisplayStr("Unable to Read RTCC\r\n");


				break;										// exit here, cannot display anthing more
			}
			else
			{
				IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
				AddDisplayStr(szfnDisplayStr);

			}
			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 1:
			// calculate and display Sun Position
			// ==> NOTE: CalculateSunPosition() needs to be broken down into a FSM/sequncer because it runs too long
			IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
			strcpy(szfnDisplayStr, "SPA\t");
			SPA_Backtrack_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);		// format for display
			AddDisplayStr(szfnDisplayStr);
			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 2:
			// convert SPA orientation to Local Orientation
			ConvertSPAtoLocalOrientation(&SPAOrientation);
			// display SPA Local orientation
			strcpy(szfnDisplayStr, "SPA Local ");
			Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);	// format for display
			AddDisplayStr(szfnDisplayStr);
			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 3:
#ifdef USE_BACKTRACKING
			// calculate Backtracking Adjustments
			IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
			if (ptrRAM_SystemParameters->bBacktrackingEnabled == TRUE)
			{
				// display orientation adjusted for Backtracking
				strcpy(szfnDisplayStr, "Backtrack ");
				Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);	// format for display
				AddDisplayStr(szfnDisplayStr);
			}
#endif	//  USE_BACKTRACKING
			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 4:
			// bounds check and adjust Orientation for limits or stow positions
			IGNORE_RETURN_VALUE PanelPositionFSM(&SPAOrientation);

			strcpy(szfnDisplayStr, "SetPt\t");

			Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);	// format for display
			AddDisplayStr(szfnDisplayStr);

			fgbStreamSunAnglesState = 0;					// restart sequence
			break;
		}
		break;
	}
	case BMS_STATUS:
	{
		int nStatus=0;
		eStreamLine = STREAM_LINE;				// mark display of {some} line of data
//		float ftemp;
		switch(fgbStreamSunAnglesState)
		{
		case 0:

			strcpy(szfnDisplayStr, "BMS : ");

			strcat(szfnDisplayStr, ftoa2(BMS_V, &nStatus));

			strcat(szfnDisplayStr, "\t M1C : ");

			strcat(szfnDisplayStr, ftoa2(M1C, &nStatus));

			strcat(szfnDisplayStr, "\t Temp : ");

			strcat(szfnDisplayStr, ftoa2(BTemp, &nStatus));

			AddDisplayStr(szfnDisplayStr);

			++fgbStreamSunAnglesState;						// bump to next line
			fgbStreamSunAnglesState = 0;
			break;

		case 1:
			AddDisplayStr(szfnDisplayStr);
			++fgbStreamSunAnglesState;						// bump to next line

			break;

		case 2:
			AddDisplayStr(szfnDisplayStr);

			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 3:
			AddDisplayStr(szfnDisplayStr);

			++fgbStreamSunAnglesState;						// bump to next line
			break;

		case 4:
			AddDisplayStr(szfnDisplayStr);
			fgbStreamSunAnglesState = 0;					// restart sequence
			break;
		}
		break;
	}
	case RUNTIME_ERROR_STATUS:
	{
		// display the contents of the runtime error buffer
		// note that we do not actually remove anything from the buffer; we just bump a display index that is initialized to 0
		if ((fgwErrorDisplayIndex < ERROR_BUFFER_SIZE) && (pgwRuntimeErrorBuffer[fgwErrorDisplayIndex] != ZERO))
		{
			eStreamLine = STREAM_LINE;				// mark display of {some} line of data
			AddDisplayStr("0x");					// leading characters


			WORDtoHexASCIIstr(pgwRuntimeErrorBuffer[fgwErrorDisplayIndex], szfnDisplayStr);
			AddDisplayStr(szfnDisplayStr);

			++fgwErrorDisplayIndex;					// bump display index
			bRetVal = TRUE;							// data to display
		}
		else
		{
			eStreamLine = STREAM_LAST_LINE;			// mark display of LAST line of available data

			AddDisplayStr(" -end-");				// really not a good way to do things; does not wait for completion

			ResetRuntimeError();					// reset the buffer pointer; has the effect of clearing the Runtime Error buffer

			bRetVal = FALSE;						// no more data to display
		}
	}
	break;

	default:
		bRetVal = FALSE;							// no more data to display
		break;

	}			// end switch

	AddDisplayNewLine();								// add line terminator

	DisplayStr();										// start display (serial output) of line

	return bRetVal;										// TRUE if more data to display, FALSE if last line
}


// *****************************************************************************
//				Current Orientation Display functions
// *****************************************************************************

void CurrentMechanicalOrientation_Format(char *ptrOutputStr)
{
	int nStatus = 0;
	SmartTrakOrientation Orientation;

	CurrentMechanicalOrientation_Read(&Orientation);
	AddDisplayStr("\t");
#ifdef USE_AZIMUTH
	AddDisplayStr("Az: ");
#endif
#ifdef USE_ELEVATION
	AddDisplayStr(" E1: ");
#endif
	DisplayStr();
}

void Orientation_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation)
{
	int nStatus = 0;
	// start string buffer
	AddDisplayStr("\t");
#ifdef USE_AZIMUTH
	AddDisplayStr("Az: ");
#endif
#ifdef USE_ELEVATION
	AddDisplayStr(" E1: ");
#endif
	DisplayStr();
}

void CurrentLocalOrientation_Format(char *ptrOutputStr)
{
	LOCAL ARRAY char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	int nStatus = 0;
	ARRAY char strtemp[20];
	SmartTrakOrientation Orientation;
//	AddDisplayStr("\t");


#ifdef USE_AZIMUTH
//	AddDisplayStr("Az: ");
//	strcat(szfnDisplayStr, (const char *)ftoa2(Orientation.fAzimuth, (int *)&nStatus));
//	AddDisplayStr( " Ticks: ");
	WORDtoASCIIstr((WORD)pgulMoveTotalMSICtr[AXIS_AZIMUTH], WORD_WIDTH, strtemp);
	//INT32StoASCIIstr(Orientation.lAzimuthPositionTicks, INT32S_WIDTH, strtemp);

#endif
#ifdef USE_ELEVATION
	AddDisplayStr( "  El: ");
	strcat(szfnDisplayStr, (const char *)ftoa2(Orientation.fElevation, (int *)&nStatus));
	AddDisplayStr( " Ticks: ");
#endif
//	DisplayStr();
}


// SPA format always displays both axis, because both are part of the calculation
void SPA_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation)
{
	int nStatus = 0;


	strcpy(ptrOutputStr, "\t");		// start string buffer		

	strcat(ptrOutputStr, "Az: ");	// start string buffer
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fAzimuth, (int *)&nStatus));
	strcat(ptrOutputStr, "  El: ");
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fElevation, (int *)&nStatus));


	DisplayStr();
}

#ifdef USE_BACKTRACKING
void SPA_Backtrack_Format(char *ptrOutputStr, SmartTrakOrientation *ptrOrientation)
{
	int nStatus = 0;


	strcpy(ptrOutputStr, "\tAz Tilt: ");
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fAzimuthTiltAngleDegrees, (int *)&nStatus));
	strcat(ptrOutputStr, " ModuleTlt: ");
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fModuleTiltAngleDegrees, (int *)&nStatus));
	strcat(ptrOutputStr, " Backtrk: ");
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fBackTrack, (int *)&nStatus));
	strcat(ptrOutputStr, " Sun El: ");
	strcat(ptrOutputStr, (const char *)ftoa2(ptrOrientation->fSunElevationAngleDegrees, (int *)&nStatus));
	DisplayStr();

}
#endif	// USE_BACKTRACKING


int UpdatedMainMenu(char cKeystroke)
{
	int nRetVal = COMMAND_SELECTED;
	int nStatus;
	char szfnDisplayStr[DISPLAY_LINE_SIZE + 1];
	RTCC_DATE_TIME CurrentDateTime;
	PTR_RTCC_DATE_TIME ptrDateTime = (PTR_RTCC_DATE_TIME) &CurrentDateTime;
	SmartTrakOrientation SPAOrientation;
	//	BYTE ucPrevTrackingMode;
	float el=0;
	char temp[20];
	char szReturnStr3[DISPLAY_LINE_SIZE + 1];
	switch(cKeystroke)
	{
	case '0':
		break;
	case '1':				// Run UP
#ifdef USE_ELEVATION
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_UP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_UP_SWITCH_OPEN_EVENT);				// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run UP (Forward)");
			memset(dispstr,0,sizeof(dispstr));
			strcpy(dispstr,"Run UP (Forward)");
			////////							HAL_UART_Transmit(UARTid,(unsigned char *)dispstr,strlen(dispstr),10);
			//							while (app_uart_put((uint8_t)dispstr) != NRF_SUCCESS);
			//		app_uart_put((uint8_t)dispstr);
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run UP only available in MANUAL mode");
			memset(dispstr,0,sizeof(dispstr));
			strcpy(dispstr,"Run UP only available in MANUAL mode");
			////////					HAL_UART_Transmit(UARTid,(unsigned char *)dispstr,strlen(dispstr),10);
			//		while (app_uart_put((uint8_t)dispstr) != NRF_SUCCESS);
			//		app_uart_put((uint8_t)dispstr);
			DisplayStr();

		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();

#endif
		break;

	case '2':				// Run EAST
#ifdef USE_AZIMUTH
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_EAST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run EAST (Reverse)");
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run EAST only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '3':				// Run WEST
#ifdef USE_AZIMUTH
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_WEST_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run WEST (Forward)");
			DisplayStr();
		}
		else
		{
			AddDisplayStrAndNewLine("Run WEST only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '4':				// Run DOWN
#ifdef USE_ELEVATION
		if (ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
			BITSET(efSwitchEvents, EF_SW_STOW_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
			AddDisplayStrAndNewLine("Run DOWN (Reverse)");
			DisplayStr();
			nRetVal = COMMAND_SELECTED;
		}
		else
		{
			AddDisplayStrAndNewLine("Run DOWN only available in MANUAL mode");
			DisplayStr();
		}
		nRetVal = COMMAND_SELECTED;
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '5':				// Find End Points
		BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_CLOSED_EVENT);	// CLOSED is processed first, same event as button push
		BITSET(efSwitchEvents, EF_SW_CALIBRATE_SWITCH_OPEN_EVENT);
		AddDisplayStrAndNewLine("Calibrate (Find End Points)");
		DisplayStr();
		nRetVal = COMMAND_SELECTED;
		break;

	case '6':				// Set Current Orientation to Sun Position
		// this is implemented by calculating the offset from the Mechanical Orientation to the SPA Orientation
		DisplayMessage("Current Orientation = Sun Position\r\n", WAIT_FOR_DISPLAY);

		// get, display current date and time
		if(ReadRTCCDateTime(ptrDateTime) != TRUE)
		{
			DisplayMessage("Unable to Read RTCC\r\n", WAIT_FOR_DISPLAY);
		}
		IGNORE_RETURN_VALUE FormatRTCCDateTime(szfnDisplayStr, ptrDateTime);
		DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);
		// calculate and display Sun Position, global coordinates
		/*fgSPAMove.SunPositionState = */ IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
		strcpy(szfnDisplayStr, "SPA ");
		Orientation_Format((szfnDisplayStr + strlen(szfnDisplayStr)), &SPAOrientation);	// format for display
		DisplayMessage(szfnDisplayStr, WAIT_FOR_DISPLAY);

		// convert SPA orientation to local orientation
		ConvertSPAtoLocalOrientation(&SPAOrientation);

		// display new Local orientation
		strcpy(szfnDisplayStr, "Local ");
		Orientation_Format(szfnDisplayStr + strlen(szfnDisplayStr), &SPAOrientation);	// format for display
		AddDisplayStrAndNewLine(szfnDisplayStr);

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

		nRetVal = COMMAND_SELECTED;
		break;

	case '7':				// fAZ_Offset
		memset(dispstr,0,sizeof(dispstr));
		AddDisplayStr("fAZ_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_Offset, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_AZ_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		DisplayStr();
		break;

	case '8':				// fEL_Offset
#ifdef USE_ELEVATION
		AddDisplayStr("fAZ_Offset: ");
		AddDisplayStr((const char *)ftoa2(ptrRAM_SystemParameters->fAZ_Offset, &nStatus));
		AddDisplayStr("  _");

		nRetVal = PARAMETER_SELECTED;
		eParameter = PARAMETER_EL_OFFSET;
		eParameterType = PARAMETER_TYPE_FLOATING_PT;
		DisplayStr();										// start display (serial output) of line
#else
		AddDisplayStrAndNewLine("Not Available");
		DisplayStr();
#endif
		break;

	case '9':				// Current Local Orientation
#ifdef USE_MMA8452Q_INCLINOMETER
		// re-Initialize Inclinometer Averaging by refilling the averaging array (twice).
		if (Init_InclinometerSampleAveraging() != TRUE)
		{
			DisplayMessage("Unable to Read Inclinometer\r\n", WAIT_FOR_DISPLAY);
		}
#endif	//  USE_MMA8452Q_INCLINOMETER
		AddDisplayStr("Current Local Orientation (degrees): ");
		CurrentLocalOrientation_Format(szfnDisplayStr);
		AddDisplayStrAndNewLine(szfnDisplayStr);
		DisplayStr();
		nRetVal = COMMAND_SELECTED;

		break;

	case 'a':				// SPA Calculated Orientation
	{
		if (ReadRTCCDateTime(ptrDateTime) != TRUE)
		{
			//  RuntimeError(RX_MSG_ERROR_RTCC_READ_ERROR);
			strcpy(szReturnStr3, "0:0:0:0:0:0:0:0:0:0\n\r");
			DisplayMessage( szReturnStr3, WAIT_FOR_DISPLAY);
			return FALSE;
		}
		else
		{
			ADD2RTCCDateTime(szReturnStr3, ptrDateTime);
		}
		strcat(szReturnStr3, ":");
		IGNORE_RETURN_VALUE CalculateSunPosition(&SPAOrientation, ptrDateTime);
		el=SPAOrientation.fElevation;
		ConvertSPAtoLocalOrientation(&SPAOrientation);
		strcat(szReturnStr3,(const char *)ftoa2(SPAOrientation.fAzimuth, &nStatus));
		strcat(szReturnStr3, ":");
		if(ptrRAM_SystemParameters->ucTracking_Mode == MODE_TRACKING)
		{
			IGNORE_RETURN_VALUE AdjustForBacktracking(&SPAOrientation);
			PanelPositionFSM(&SPAOrientation);
			strcat(szReturnStr3,(const char *)ftoa(SPAOrientation.fAzimuth , &nStatus));
			strcat(szReturnStr3, ":");
		}
		else
		{
			if((MAN_WEST == 1)||(MAN_EAST == 1))
			{
				strcat(szReturnStr3,(const char *)ftoa(pgfMS_ManDistanceDegrees[MOTOR_AZIMUTH], &nStatus));
				strcat(szReturnStr3, ":");
			}
			else
			{
				//strcat(szReturnStr,);
				strcat(szReturnStr3, "0.0:");
			}
		}

#ifdef USE_MMA8452Q_INCLINOMETER
		// strcat(szReturnStr, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
		// strcat(szReturnStr, ":");

		strcat(szReturnStr3, (const char *)ftoa(pgAngleAverage.fX_Angle , &nStatus));
		strcat(szReturnStr3, ":");
#endif

		//IGNORE_RETURN_VALUE ADD2RTCCDateTime(szReturnStr + strlen(szReturnStr), ptrDateTime);
		//strcat(szReturnStr, ":");
		//4digits    x1x2x3x4
		//    x1- 0POWER_UP/1INIT/2TRACKING/3HOLD/4NIGHT_STO/5WIND_STOW
		//    x2- 1MAN/2AUTO     x3- 0stopped/1West/2East
		//    x4- 0MotorStooped/1Motor running
		BYTEtoASCIIstr(SPS, temp);
		if(((int)SPS < 1) || ((int)SPS > 6))
			strcat(szReturnStr3,"07");
		else
			strcat(szReturnStr3, temp);
		if(ptrRAM_SystemParameters->ucTracking_Mode == MODE_MANUAL)
		{
			if(MAN_EAST == 1)
				strcat(szReturnStr3, "12");  //east
			else if(MAN_WEST == 1)
				strcat(szReturnStr3, "11");  //West
			else if(MAN_STOW == 1)
				strcat(szReturnStr3, "13");  //stow
			else
				strcat(szReturnStr3, "10");
		}
		else
		{
			if(bTrackerDirection == 1)
				strcat(szReturnStr3, "21");  //West
			else if(bTrackerDirection == 2)
				strcat(szReturnStr3, "22");  //east
			else
				strcat(szReturnStr3, "20");  //stop
		}
		if (IsCommandComplete(AXIS_AZIMUTH) IS_TRUE)
			strcat(szReturnStr3, "0\n\r");
		else
			strcat(szReturnStr3, "1\n\r");
	}
	DisplayMessage(szReturnStr3, WAIT_FOR_DISPLAY);
	break;

	case 'b':				// clear Orientation
		AddDisplayStrAndNewLine("Clear Current Orientation from RTCC NV RAM and MCU RAM Copy");
		CurrentPosition_Clear(MOTOR_AZIMUTH);								// clear the MSI Tick counters
		CurrentPosition_Clear(MOTOR_ELEVATION);
		// ClearRTCCRAMOrientation() handles #ifdef USE_DS3232_RTCC internally
//		IGNORE_RETURN_VALUE ClearRTCCRAMOrientation();						// clear MCU RAM copy of Orientation, then Update the RTCC NV RAM copy of Orientation
		nRetVal = COMMAND_SELECTED;
		DisplayStr();
		break;

	case 'c':				// clear Orientation
#ifdef USE_DS3232_RTCC
		AddDisplayStrAndNewLine("Clear Entire RTCC NV RAM, Init MCU RAM Copy");
		DisplayStr();
		IGNORE_RETURN_VALUE ClearRTCCRAM();
#else
		AddDisplayStrAndNewLine("RTCC not available. Init MCU RAM Copy ONLY");
		DisplayStr();
#endif
		// clear RTCC NV RAM parameter table, current orientation
		// InitRTCCRAMParameterTable() handles #ifdef USE_DS3232_RTCC internally
/*		if(InitRTCCRAMParameterTable() != MEMORY_INITIALIZED)			// re-initialize tables (orientation will be 0, 0)
		{
			AddDisplayStrAndNewLine("RAM Initialization Failure");
			DisplayStr();
		}*/
		nRetVal = COMMAND_SELECTED;
		break;

	case 'd':				// ucTracking_Mode
		BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
		BITSET(efSwitchEvents, EF_SW_SERVICE_SWITCH_OPEN_EVENT);			// OPEN will be processed AFTER CLOSED, has the effect of push and release
		AddDisplayStrAndNewLine("Auto/Manual Mode");
		nRetVal = COMMAND_SELECTED;
		AddDisplayStr("Prev ucTracking_Mode: ");
		//			ucPrevTrackingMode = ptrRAM_SystemParameters->ucTracking_Mode;		// keep track of previous tracking mode
		BYTEtoHexASCIIstr(ptrRAM_SystemParameters->ucTracking_Mode, szfnDisplayStr);
		AddDisplayStr(szfnDisplayStr);
		switch(ptrRAM_SystemParameters->ucTracking_Mode)
		{
		case MODE_MANUAL:
			AddDisplayStr(" Manual Mode");
			break;

		case MODE_TRACKING:
			AddDisplayStr(" AUTO SPA Tracking");
			break;

		case MODE_NIGHT_STOW:
		case MODE_WIND_STOW:
		default:
			AddDisplayStr(" Unknown/Error");
			break;
		}
		DisplayStr();										// start display (serial output) of line
		break;

		case 'e':				// write parameter table to flash
			WriteFlashParameterTable();		// ==> should have a return value
			AddDisplayStrAndNewLine("Table Write Complete");
			DisplayStr();
			break;

		case 'f':				// display runtime errors
			eCurrentStatus = RUNTIME_ERROR_STATUS;
			fgwErrorDisplayIndex = 0;					// reset error display index
			nRetVal = DATA_STREAM_SELECTED;
			break;

		case 'g':				// Utility Menu
			eCurrentMenu = UTILITY_MENU;
			nRetVal = MENU_SELECTED;
			break;

		case 'h':				// Remote Mode
			AddDisplayStrAndNewLine("\x1B[2J\x1B[0;0H Remote Mode: ");
			DisplayStr();										// start display (serial output) of line

			ptrRAM_SystemParameters->eSerialOutputMode = SER_MODE_REMOTE;
			eSerialOutputMode = SER_MODE_REMOTE;								// update program global copy
			////			ChangeSerialBaudRate(SERIAL_REMOTE_UART, DESIRED_REMOTE_BAUDRATE);
			WriteFlashParameterTable();
			//			nRetVal = PARAMETER_SELECTED;
			//			eParameter = PARAMETER_AZ_OFFSET;
			//			eParameterType = PARAMETER_TYPE_DEC;
			break;


		case 's':				// stop
			// make sure there is an active motor to stop
			if (IsMoveSequenceComplete() IS_FALSE)
			{
				// there is NO STOP switch at present, but with ButtonProcessingFSM() in ST_BT_PROC_MOVING,  closing ANY switch will cause a STOP
				BITSET(efVirtualSwitchEvents, EF_VSW_STOP_SWITCH_CLOSED_EVENT);			// CLOSED is processed first, same event as button push
				// no need for an OPEN even, STOP clears all switch and virtual switch events
				AddDisplayStrAndNewLine("All STOP!");
				DisplayStr();
				nRetVal = COMMAND_SELECTED;
			}
			break;

		case 'r':				// reset
			BITSET(efSwitchEvents, EF_SW_RESET_SWITCH_CLOSED_EVENT);
			break;

	}

	eResultLine = RESULT_SINGLE_LINE;			// only one result line to display
	return nRetVal;

}


// end of MenuFSM.c
