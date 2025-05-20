// *************************************************************************************************
//										S m a r t T r a k . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Project-Wide Definitions
//
// *************************************************************************************************

#ifndef __SMARTTRAK_H__
#define	__SMARTTRAK_H__

#include "Debug.h"
// *****************************************************************************
// *****************************************************************************
// Section: Project Macros
// *****************************************************************************
// *****************************************************************************

#define	NUM_MOTORS		2

enum tagMotors
	{
		MOTOR_NONE = -1,
		MOTOR_AZIMUTH = 0,
		MOTOR_ELEVATION
	};

#define	NUM_AXIS		2

enum tagAxis
	{
		AXIS_NONE = -1,
		AXIS_AZIMUTH = 0,
		AXIS_ELEVATION
	};


enum tagSystemErrors
{
	SYSTEM_ERROR_NONE = SYSTEM_ERROR_BASE,
	SYSTEM_ERROR_UNEXPECTED_TICK,			// 1 unexpected timer tick event
	SYSTEM_ERROR_UNEXPECTED_EVENT,			// 2 unexpected event
	SYSTEM_ERROR_INVALID_STATE,				// 3 not a valid state
	SYSTEM_ERROR_INVALID_SUBSTATE,			// 4 not a valid state
	SYSTEM_ERROR_UNKNOWN_COMMAND,			// 5 not a valid command
	SYSTEM_ERROR_FLASH_NOT_PRESENT,			// 6 SPI Flash not found
	SYSTEM_ERROR_FLASH_PARAMETER_TABLE,		// 7 System Parameter table checksum or other error
	SYSTEM_ERROR_RAM_PARAMETER_TABLE,		// 8 App RTCC RAM Parameter table checksum or other error

	SYSTEM_ERROR_UNPROCESSED_EVENT = SYSTEM_ERROR_BASE + 0x0F
};


// the standard cast from float to int always rounds DOWN
// this offset is added to floats so that fractions less than 0.5 will still round down,
// but fractions from 0.50 to 0.99 will round UP
#define	CAST_FLOAT_ROUNDING_OFFSET	(float)0.500

#define	LARGE_ERROR_BUFFER

#define	UART_CNT					2

//#if defined (PLATFORM_PIC32_SK)
//	#define	RS232_UART					UART2
//#elif defined (PLATFORM_SMARTTRAK_V1)
//	#define	RS232_UART1					UART1
//	#define	RS232_UART2					UART2
//#else
//	#error Platform must be defined
//#endif		// PLATFORM_SMARTTRAK_V1

//#define	SERIAL_MENU_UART			RS232_UART1
//#define	SERIAL_REMOTE_UART			RS232_UART2	// may be platform dependent, PIC32SK supports two UARTs

//#define DESIRED_MENU_BAUDRATE		(9600)		// The desired BaudRate;
//#define DESIRED_RS485_BAUDRATE		(9600)
//#ifdef USE_SLOW_REMOTE_BAUDRATE
//	#define DESIRED_REMOTE_BAUDRATE		(9600)	// XBee baud rate, nominal for new modules
//#elif defined(USE_FAST_REMOTE_BAUDRATE)
//	#define DESIRED_REMOTE_BAUDRATE		(9600)	// XBee baud rate to match serial menus
//#endif

//#define	DEFAULT_REMOTE_BAUDRATE		(9600)		// XBee baud rate after 'Restore Defaults' or equivalent command

#if defined(LOCATION_HYDERABAD_AP_IN)
	#if defined (DEFINE_GLOBALS)				// only output warning for Main.c
		#warning Using LOCATION_HYDERABAD_AP_IN
	#endif
	#define	LOCAL_LATITUDE		17.3667			// south of Tropic of Cancer
	#define	LOCAL_LONGITUDE		78.4667
	#define	LOCAL_ALTITUDE		542				// meters
	#define LOCAL_TIMEZONE		5.5				// relative to GMT
#elif defined(LOCATION_FREMONT_USA)
	#if defined (DEFINE_GLOBALS)				// only output warning for Main.c
		#warning Using LOCATION_FREMONT_CA_USA
	#endif
	#define	LOCAL_LATITUDE		37.5422
	#define	LOCAL_LONGITUDE		-122.015
	#define	LOCAL_ALTITUDE		56				// meters, at sea level!
	#define LOCAL_TIMEZONE		-8.0			// EDT, relative to GMT

#elif defined(LOCATION_BEVERLY_MA_USA)
	#if defined (DEFINE_GLOBALS)				// only output warning for Main.c
		#warning Using LOCATION_BEVERLY_MA_USA
	#endif
	#define	LOCAL_LATITUDE		42.5583
	#define	LOCAL_LONGITUDE		-70.8806
	#define	LOCAL_ALTITUDE		10				// meters, at sea level!
	#define LOCAL_TIMEZONE		-4.0			// EDT, relative to GMT
#else
	#warning No Default Location Defined
	#define	LOCAL_LATITUDE		0.0
	#define	LOCAL_LONGITUDE		0.0
	#define	LOCAL_ALTITUDE		0.0
	#define LOCAL_TIMEZONE		0.0
#endif

#define	NIGHT_STOW_EL_THRESHOLD_DEGREES		-1.0	// minimum elevation value which allows sun tracking

enum tagTrackingMode
{
	MODE_MANUAL = 0,
	MODE_TRACKING,
	MODE_NIGHT_STOW,
	MODE_WIND_STOW
};

enum tagSerialOutputMode
{
	SER_MODE_UNINITIALIZED = 0,
	SER_MODE_MENU,				// menus and responses
	SER_MODE_REALTIME,			// realtime output, primarily firmware status
	SER_MODE_STREAM,			// menu selected streaming
	SER_MODE_REMOTE				// remote commands
};


// structure used to store current, local orientation
typedef struct
{
	INT32 lAzimuthPositionTicks;	// Azimuth, Units: MSI ticks
	INT32 lElevationPositionTicks;	// Elevation, Units: MSI ticks

	float	fAzimuth;				// Float Azimuth. Units: degrees
	float	fElevation;				// Float Elevation. Units: degrees

	#ifdef USE_BACKTRACKING
		BOOL	bBacktrackingActive;			// Bool R/W		True: backtracking is enabled
		float	fSunElevationAngleDegrees;
		float	fSunShadowStartAngleDegrees;	// presently just a fixed value
		float	fSunShadowStartHeight;			// presently just a fixed value, needed here?
		float	fAzimuthTiltAngleDegrees;		// calculated value
		float	fModuleTiltAngleDegrees;		// calculated value
		float	fBackTrack;		// calculated value
	#endif	// USE_SINGLE_POLAR_AXIS

} SmartTrakOrientation, *PTR_ORIENTATION;


// Non-volatile SYSTEM flash parameters (this must be 16bit word aligned)
// A copy of this structure is stored in internal or external FLASH memory
typedef struct
{
    UINT16		unChecksum;					// table checksum as of last write
    UINT16		unMagic;					// start of table marker

	// ****************************
	//		unit location
	// ****************************
	float		fLatitude;					// R/W, In degrees. Range = +90 degrees to -90 degrees, Positive for north of the equator.
	float		fLongitude;					// R/W, In degrees. Range = -180 to +180deg, Positive for east of GMT
	float		fAltitude;					// Float R/W In meters. Above sea level.
	float		fRefraction;				// Float R/W No units. Value is around 1.
	float		fTimeZone;					// Float R/W Offset from GMT (like 5.5 for India)

	enum tagTrackingMode ucTracking_Mode;	// (enum) R/W Values: Manual, Tracking, (Night Stow, Wind Stow)
											// TBD: make it read only? May need write permission for testing purpose
	// ****************************
	//			Azimuth
	// ****************************
//    UINT16		unAZ_PulsesPerRot;			// Int R/W Motor setting. Number of hall pulses to complete 360deg rotation of the azimuth drive.
	float		fAZ_Offset;					// Float R/W Azimuth offset from 0.0 at due south Units: degrees
	float		fAZ_SoftLimit_Reverse;		// Float R/W Azimuth soft limit REVERSE (minimum). Units: degrees
	float		fAZ_SoftLimit_Forward;		// Float R/W Azimuth soft limit FORWARD (maximum). Units: degrees
	float		fAZ_DeadBand;				// Float R/W TBD
	float		fAZ_NightStowThreshold;		// Float R/W Elevation threshold to move to/exit NightStow
	float		fAZ_NightStowPosition;		// Float R/W Default = 0 (remain in place)
	float		fAZ_WindStowPosition;		// Float R/W Default = 0 (remain in place)
//	float		fAZ_SetPoint;				// Float R/W This comes from SPA_calc, but limited by soft limits
//	float		fAZ_PVpos;					// Float R only This is the present position of the PV panel. This is calculated based on AZ_PulseCount.
//	UINT16		unAZ_PulseCount;			// Int R only This is the number of hall pulses.
//	BOOL		bAZ_En_AutoCal;				// Bool R/W
//	BOOL		bAZ_AutoMove;				// Bool, R/W	True: Move the motor to AZ_SetPoint
											//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
//	float		fAZ_cal_param1;				// Float R/W TBD: could be needed for cal correction
//	float		fAZ_cal_param2;				// Float R/W TBD: could be needed for cal correction

	// ****************************
	//			Elevation
	// ****************************
//    UINT16		unEL_PulsesPerRot;			// Int R/W Motor setting. Number of hall pulses to complete *deg rotation of the elevation drive.
	float		fEL_Offset;					// Float R/W Elevation offset from 45.0 at center Units: degrees
	float		fEL_SoftLimit_Reverse;		// Float R/W Elevation soft limit Reverse (minimum). Units: degrees
	float		fEL_SoftLimit_Forward;		// Float R/W Elevation soft limit Forward (maximum). Units: degrees
	float		fEL_DeadBand;				// Float R/W TBD
	float		fEL_NightStowThreshold;		// Float R/W Elevation threshold to move to/exit NightStow
	float		fEL_NightStowPosition;		// Float R/W Default = 0 (remain in place)
	float		fEL_WindStowPosition;		// Float R/W Default = 0 (remain in place)
//	float		fEL_SetPoint;				// Float R/W This comes from SPA_calc, but limited by soft limits
//	float		fEL_PVpos;					// Float R only This is the present position of the PV panel. This is calculated based on EL_PulseCount.
//	UINT16		unEL_PulseCount;			// Int R only This is the number of hall pulses.
//	BOOL		bEL_En_AutoCal;				// Bool R/W
//	BOOL		bEL_AutoMove;				// Bool R/W		True: Move the motor to EL_SetPoint
											//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
//	float		fEL_cal_param1;				// Float R/W TBD: could be needed for cal correction
//	float		fEL_cal_param2;				// Float R/W TBD: could be needed for cal correction

	// ****************************
	//		Backtracking
	// ****************************

	#ifdef USE_BACKTRACKING
		BOOL		bBacktrackingEnabled;		// Bool R/W		True: backtracking is enabled
		float		fPanelShadowStartAngleDegrees;	// S_S_P
		float		fSunShadowStartAngleDegrees;	// S_S_S
		float		fSunShadowStartHeight;		// new 1 Apr 14 per SmartTrak
        #endif
        // ****************************
	//		SingleAxis Softlimits
	// ****************************
        float           fSingle_SoftLimit_Forward;
        float           fSingle_SoftLimit_Reverse;

        // ************************************
	//		SingleAxis days limits
	// ****************************
        float           fSingle_start_date;
        float           fSingle_stop_days;
        BYTE            fSingle_days_update;
	

	// ****************************
	//		System
	// ****************************

	enum tagSerialOutputMode eSerialOutputMode;


	// ****************************
	//		End Markers
	// ****************************
	UINT16		unUpdateCtr;				// count of number of times table has been updated, for debugging
	UINT16		unEndMarker;				// end of structure marker, really for debugging


} FLASH_SYSTEM_PARAMETERS, *PTR_FLASH_SYSTEM_PARAMETERS;


/* Total size of SYSTEM and APPLICATION attribute (flash) table */
#define FLASH_SYS_PARAMETERS_TABLE_SIZE		sizeof(FLASH_SYSTEM_PARAMETERS)

// change the "Magic Number" whenever the FLASH_SYSTEM_PARAMETERS structure is modified, to FORCE writing a new default table to
#define	DEFAULT_SYS_FLASH_MAGIC_NUMBER		(0xB400 | FLASH_SYS_PARAMETERS_TABLE_SIZE)

// end of structure marker default values, for debugging
#define	DEFAULT_TABLE_UPDATE_CTR		0x00
#define	DEFAULT_TABLE_END_MARKER		0x55AA

#define	SPI_FLASH_TABLE_ADDRESS		(0x00000000)

// boundary values, for limit checking
#define	MIN_LATITUDE	-90.0
#define	MAX_LATITUDE	90.0
#define	MIN_LONGITUDE	-180.0
#define	MAX_LONGITUDE	180.0
#define	MIN_ALTITUDE	-414.0		// Dead Sea, Israel
#define	MAX_ALTITUDE	8848.0		// Mt Everest, Nepal
#define	MIN_REFRACTION	0.0
#define	MAX_REFRACTION	1.1
#define	MIN_TIMEZONE	-23.5
#define	MAX_TIMEZONE	23.5

#define	DEFAULT_PANEL_SHADOW_START_ANGLE_DEGREES	45.0	// S_S_P
#define	DEFAULT_SUN_SHADOW_START_ANGLE_DEGREES		60.0	// S_S_S  was 62.0
#define DEFAULT_SUN_SHADOW_START_HEIGHT				0.0		// ==>> meaningful?


#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)
	GLOBAL FLASH_SYSTEM_PARAMETERS		RAM_SystemParameters;								// instantiation of RAM copy of System Parameters
	GLOBAL_INIT FLASH_SYSTEM_PARAMETERS	*ptrRAM_SystemParameters = &RAM_SystemParameters;	// pointer to RAM copy of System Parameters
	GLOBAL_INIT BOOL gbSPIFlashPresent = FALSE;
#elif defined (DEFINE_EXTERNS)
	GLOBAL FLASH_SYSTEM_PARAMETERS		*ptrRAM_SystemParameters;
	GLOBAL BOOL	gbSPIFlashPresent;
#endif


typedef struct
{
    UINT16		unChecksum;					// table checksum as of last write
    UINT16		unMagic;					// start of table marker

	// ****************************
	//		System Orientation
	// ****************************
	// last (most recent) system orientation, updated every MSI tick
	INT32		lAzimuthAtSPA;
	INT32		lElevationAtSPA;

	// ****************************
	//		Sun Orientation
	// ****************************
	float		fSPACalculation_AZ;			// Float R only Output of SPA calc.
	float		fSPACalculation_EL;			// Float R only Output of SPA calc.
	float		fSetPoint_AZ;				// Float R/W This comes from SPA_calc, but limited by soft limits
	float		fSetPoint_EL;				// Float R/W This comes from SPA_calc, but limited by soft limits

//	float		fSPA_sunrise;				// Float R only Output of SPA calc.
//	float		fSPA_sunset;				// Float R only Output of SPA calc.
//	float		fSPA_param1;				// Float R only TBD
//	float		fSPA_param2;				// Float R only TBD

	UINT16		unUpdateCtr;				// count of number of times table has been updated, for debugging
	UINT16		unEndMarker;				// end of structure marker, really for debugging

} RTCC_RAM_APP_PARAMETERS;


// this small structure is used to save the Mechanical Orienation on every MSI tick,
// because the above structure takes too long to write so frequently
typedef struct
{
	// ****************************
	//		System Orientation
	// ****************************
	// last (most recent) system orientation, updated every MSI tick
	INT32		lLastAzimuth;
	INT32		lLastElevation;

} MECHANICAL_ORIENTATION_TICKS, *PTR_MECHANICAL_ORIENTATION_TICKS;


/* Total size of SYSTEM and APPLICATION attribute (flash) table */
#define RTCC_RAM_APP_PARAMETERS_TABLE_SIZE	sizeof(RTCC_RAM_APP_PARAMETERS)
#define	MECHANICAL_ORIENTATION_TICKS_SIZE	sizeof(MECHANICAL_ORIENTATION_TICKS)

#define	DEFAULT_APP_RAM_MAGIC_NUMBER		(0x5A00 | RTCC_RAM_APP_PARAMETERS_TABLE_SIZE)

// storage offsets into RTCC RAM
#define	SMARTTRAK_APP_RAM_TABLE_OFFSET		0								// complete RTCC_RAM_APP_PARAMETERS structure, offset from start of RTCC NV RAM
#define	CURRENT_ORIENTATION_OFFSET			RTCC_RAM_APP_PARAMETERS_TABLE_SIZE	// current orientation in ticks, offset from start of RTCC NV RAM


#if defined (DEFINE_GLOBALS)
	GLOBAL_INIT RTCC_RAM_APP_PARAMETERS		RTCC_RAM_AppParameters;				// instantiation of RAM copy of System Parameters
	GLOBAL_INIT RTCC_RAM_APP_PARAMETERS		*ptrRTCC_RAM_AppParameters = &RTCC_RAM_AppParameters;

	GLOBAL_INIT MECHANICAL_ORIENTATION_TICKS		RTCC_RAM_MechanicalOrientation;			// instantiation of RAM copy of System Orientation
	GLOBAL_INIT MECHANICAL_ORIENTATION_TICKS		*ptrRTCC_RAM_MechanicalOrientation = &RTCC_RAM_MechanicalOrientation;

#elif defined (DEFINE_EXTERNS)
	GLOBAL RTCC_RAM_APP_PARAMETERS			*ptrRTCC_RAM_AppParameters;
	GLOBAL MECHANICAL_ORIENTATION_TICKS		*ptrRTCC_RAM_MechanicalOrientation;
#endif

// return values for SPI FLASH, I2C RTCC NV RAM initialization functions
enum tagMemoryInit
	{
		MEMORY_TIMEOUT = -2,
		MEMORY_NOT_PRESENT = -1,
		MEMORY_CHECKSUM_ERROR = 0,
		MEMORY_INITIALIZED = 1
	};


#if defined(PLATFORM_SMARTTRAK_V1)
	// SmartTrak LED assignments
	#define	LED_DOWN	LED4		// I/O 3
	#define	LED_EAST	LED5		// I/O 4
	#define	LED_WEST	LED6		// I/O 5
	#define	LED_UP		LED7		// I/O 6
	#define	LED_AUTO	LED8		// I/O 7
#elif defined(PLATFORM_PIC32_SK)
	#define	LED_EAST	LED5		// I/O 4
	#define	LED_WEST	LED6		// I/O 5
	#define	LED_AUTO	LED7		// I/O 6
	#define	LED_DOWN	LED8		// I/O 7	not available
	#define	LED_UP		LED8		// I/O 7	not available
#endif


enum tagMemoryInit InitSystemParameterTable(void);
void WriteFlashParameterTable(void);					// write the RAM copy of SYSTEM and APPLICATION attributes to SPI Flash

enum tagMemoryInit InitRTCCRAMParameterTable(void);
BOOL ReadRTCCRAMParameterTables(void);
void UpdateRTCCRAMParameterTable(void);
BOOL WriteRTCCRAMParameterTable(void);					// write the RAM copy of SYSTEM and APPLICATION attributes to SPI Flash
BOOL UpdateRTCCRAMOrientation(void);
BOOL ClearRTCCRAMOrientation(void);
BOOL ClearRTCCRAM(void);


#endif	/* SMARTTRAK_H */

