// *************************************************************************************************
//								S y s t e m P a r a m e t e r s . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	SystemParameter table functions
//
// *************************************************************************************************

//-----------------------------------------------------------------------------
// #include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "HardwareProfile.h"

// processor include file
//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions, default Parameter table

#include "MotionPhaseFSM.h"		// Motion Phase and Command Processing FSM functions, eMove type
#include "MotionProfile.h"		// motion profile data table, movement descriptions
//#include "MotorPWM.h"			// Motor PWM function prototypes and definitions
//#include "MotionFSM.h"			// Motion Control function prototypes and definitions
#include "MotionLimits.h"		// Motion limits, based on physical limitations

#include "SST25VF016.h"			// SPI Flash function definitions
#include "DS3232.h"				// DS3232 RTCC function definitions
#include "RTCC.h"				// DS3232 RTCC RAM function definitions

#include <string.h>				// for memcpy()

// ***********************************************
//				File Global Data
// ***********************************************
#define BUILD_S_DATE (BUILD_YEAR*10000+BUILD_MONTH*100+BUILD_DAY)

static const FLASH_SYSTEM_PARAMETERS	defaultSystemParameters =			// default valus of System Parameters
{
	.unChecksum = 0x0000,
	.unMagic = DEFAULT_SYS_FLASH_MAGIC_NUMBER,

	// ****************************
	//		unit location
	// ****************************
	.fLatitude = LOCAL_LATITUDE,		// R/W, In degrees. Range = +90 degrees to -90 degrees, Positive for north of the equator.
	.fLongitude = LOCAL_LONGITUDE,		// R/W, In degrees. Range = -180 to +180deg, Positive for east of GMT
	.fAltitude = LOCAL_ALTITUDE,		// Float R/W In meters. Above sea level.
	.fRefraction = 0.0,					// Float R/W No units. Value is around 1.
	.fTimeZone = LOCAL_TIMEZONE,		// Float R/W Offset from GMT (like 5.5 for Hyderabad AP India)
	.ucTracking_Mode = MODE_TRACKING,		// enum, default mode at startup. Always start new system or new code in MANUAL.

	// ****************************
	//			Azimuth
	// ****************************
	//	.unAZ_PulsesPerRot = 472,			// Int R/W Motor setting. Number of hall pulses to complete 360deg rotation of the azimuth drive.
		.fAZ_Offset = 0.0,					// Float R/W Azimuth offset from 0.0 at due south Units: degrees
		.fAZ_SoftLimit_Reverse = AZ_SOFT_LIMIT_DEGREES_REVERSE,	// Float R/W Azimuth soft limit minimum. Units: degrees
		.fAZ_SoftLimit_Forward = AZ_SOFT_LIMIT_DEGREES_FORWARD,	// Float R/W Azimuth soft limit maximum. Units: degrees
		.fAZ_DeadBand = 0.0,				// Float R/W TBD
		#ifdef USE_SINGLE_POLAR_AXIS
			.fAZ_NightStowPosition = AZ_NIGHTSTOW_POSITION,	// Float R/W Default = 0 (remain in place)
		#else
			.fAZ_NightStowPosition = AZ_SOFT_LIMIT_DEGREES_REVERSE,	// Float R/W Default = 0 (remain in place)
		#endif
		.fAZ_WindStowPosition = AZ_WINDSTOW_POSITION,		// Float R/W Default = 0 (remain in place)
	//	.fAZ_PVpos = 0.0,					// Float R only This is the present position of the PV panel. This is calculated based on AZ_PulseCount.
	//	.unAZ_PulseCount = 0,				// Int R only This is the number of hall pulses.
	//	.bAZ_En_AutoCal = FALSE,			// Bool R/W
	//	.bAZ_AutoMove = TRUE,				// Bool, R/W	True: Move the motor to AZ_SetPoint
											//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
	//	.fAZ_cal_param1 = 1.0,				// Float R/W TBD: could be needed for cal correction
	//	.fAZ_cal_param2 = 1.0,				// Float R/W TBD: could be needed for cal correction

	// ****************************
	//			Elevation
	// ****************************
	//	.unEL_PulsesPerRot = 472,			// Int R/W Motor setting. Number of hall pulses to complete *deg rotation of the elevation drive.
		.fEL_Offset = 0.0,					// Float R/W Elevation offset from 45.0 at center Units: degrees
		.fEL_SoftLimit_Reverse = EL_SOFT_LIMIT_DEGREES_REVERSE,		// Float R/W Azimuth soft limit minimum. Units: degrees
		.fEL_SoftLimit_Forward = EL_SOFT_LIMIT_DEGREES_FORWARD,		// Float R/W Azimuth soft limit maximum. Units: degrees
		.fEL_DeadBand = 0.0,				// Float R/W TBD
		.fEL_NightStowPosition = EL_SOFT_LIMIT_DEGREES_REVERSE,		// Float R/W Default = 0 (remain in place)
		.fEL_NightStowThreshold = NIGHT_STOW_EL_THRESHOLD_DEGREES,	// Float R/W Default = -3.0
		.fEL_WindStowPosition = 0.0,		// Float R/W Default = 0 (remain in place)
	//	.fEL_PVpos = 0.0,					// Float R only This is the present position of the PV panel. This is calculated based on EL_PulseCount.
	//	.unEL_PulseCount = 0,				// Int R only This is the number of hall pulses.
	//	.bEL_En_AutoCal = FALSE,			// Bool R/W
	//	.bEL_AutoMove = TRUE,				// Bool R/W		True: Move the motor to EL_SetPoint
											//				False: Move the motor only upon soft/hard joystick operation (soft= cmds; hard =push buttons)
	//	.fEL_cal_param1 = 0.0,				// Float R/W TBD: could be needed for cal correction
	//	.fEL_cal_param2 = 0.0,				// Float R/W TBD: could be needed for cal correction

	// ****************************
	//		Backtracking
	// ****************************
	#ifdef USE_BACKTRACKING
		.bBacktrackingEnabled = TRUE,		// Bool R/W		True: backtracking is enabled
		.fPanelShadowStartAngleDegrees = DEFAULT_PANEL_SHADOW_START_ANGLE_DEGREES,	// S_S_P
		.fSunShadowStartAngleDegrees = DEFAULT_SUN_SHADOW_START_ANGLE_DEGREES,		// S_S_S
		.fSunShadowStartHeight = DEFAULT_SUN_SHADOW_START_HEIGHT,					// new 1 Apr 14 per SmartTrak
	
        // ****************************
	//		SingleAxis
	// ****************************
        .fSingle_SoftLimit_Forward   = 38.0,
        .fSingle_SoftLimit_Reverse  = -38.0,
        #endif

         // ************************************
	//		SingleAxis days limits
	// *************************************
        .fSingle_start_date=20170313,
        .fSingle_stop_days=0,
        .fSingle_days_update = 0,

	// ****************************
	//		System
	// ****************************

	.eSerialOutputMode = SER_MODE_REMOTE,	// (enum) R/W Values: SER_MODE_MENU, SER_MODE_REMOTE


	// ****************************
	//		End Markers
	// ****************************

	.unUpdateCtr = DEFAULT_TABLE_UPDATE_CTR,
	.unEndMarker = DEFAULT_TABLE_END_MARKER

};



static const RTCC_RAM_APP_PARAMETERS	defaultRTCC_RAM_AppParameters =			// default valus of System Parameters
{
    .unChecksum = 0x0000,				// table checksum as of last write
    .unMagic = DEFAULT_APP_RAM_MAGIC_NUMBER,	// start of table marker

	// ****************************
	//		System Orientation
	// ****************************
	// system orientation prior to SPA calculation
	.lAzimuthAtSPA = 0L,
	.lElevationAtSPA = 0L,

	// ****************************
	//		Sun Orientation
	// ****************************
	.fSPACalculation_AZ = 0.0,			// Float R only Output of SPA calc.
	.fSPACalculation_EL = 0.0,			// Float R only Output of SPA calc.
	.fSetPoint_AZ = 0.0,				// Float R/W This comes from SPA_calc, but limited by soft limits
	.fSetPoint_EL = 0.0,				// Float R/W This comes from SPA_calc, but limited by soft limits
//	.fSPA_sunrise = 0.0,				// Float R only Output of SPA calc.
//	.fSPA_sunset = 0.0,					// Float R only Output of SPA calc.
//	.fSPA_param1 = 0.0,					// Float R only TBD
//	.fSPA_param2 = 0.0,					// Float R only TBD

	.unUpdateCtr = DEFAULT_TABLE_UPDATE_CTR,	// count of number of times table has been updated, for debugging
	.unEndMarker = DEFAULT_TABLE_END_MARKER		// end of structure marker, really for debugging

};


// ***********************************************
//	Forward References (local Function Declarations)
// ***********************************************
static UINT16 ChecksumSystemParameterTable(UINT8 *table);
static void ReadFlashParameterTable(void);
//void WriteFlashParameterTable(void);					// write the RAM copy of SYSTEM and APPLICATION attributes to SPI Flash

static UINT16 ChecksumRTCCRAMParameterTable(UINT8 *table);

// *************************************************************************************************
//				Load and Initialize System Parameter from Flash Storage
// *************************************************************************************************

enum tagMemoryInit InitSystemParameterTable(void)
{
	enum tagMemoryInit eRetVal = MEMORY_INITIALIZED;
	BYTE SST25_ID[4];					// <sek> 17 Aug 13 changed from char to BYTE
	UINT16	nCheckSum;

	// initialize SPI interface to the SST25 SPI flash
    SST25Init();

	// verify presence of the SPI Flash by reading the device IDs
	if (SST25ReadID(SST25_ID) IS FALSE)		// <sek> 17 Aug 13
	{
		eRetVal = MEMORY_NOT_PRESENT;
		gbSPIFlashPresent = FALSE;
		RuntimeError(SYSTEM_ERROR_FLASH_NOT_PRESENT);

		// initialize system parameters from default table
		memcpy((UINT8*)ptrRAM_SystemParameters, (UINT8*)&defaultSystemParameters, FLASH_SYS_PARAMETERS_TABLE_SIZE);
	}
	else
	{
		gbSPIFlashPresent = TRUE;
		// read the System Parameter table from SPI Flash into MCU RAM.
		ReadFlashParameterTable();
	}


	if (gbSPIFlashPresent IS TRUE)
	{
		// calculate checksum of System Parameter table
		nCheckSum = ChecksumSystemParameterTable((UINT8 *)ptrRAM_SystemParameters);

		// check for valid Attribute table
		// if	checksum is not correct (not 0)
		//		'magic' number in attributes copied from SPI flash does not match 'magic' number for this build
		//		structure end marker has moved relative to expected location for this build (which should have changed the 'magic' number
		// copy the defaultSystemParameters table to RAM, update the checksum and 'magic' number, and write out to SPI flash
		if ((nCheckSum) OR (ptrRAM_SystemParameters->unMagic IS_NOT DEFAULT_SYS_FLASH_MAGIC_NUMBER) OR (ptrRAM_SystemParameters->unEndMarker IS_NOT DEFAULT_TABLE_END_MARKER))
		{
			eRetVal = MEMORY_CHECKSUM_ERROR;
			RuntimeError(SYSTEM_ERROR_FLASH_PARAMETER_TABLE);

			// Replace the current RAM copy of the system parameters from default table
			memcpy((UINT8*)ptrRAM_SystemParameters, (UINT8*)&defaultSystemParameters, FLASH_SYS_PARAMETERS_TABLE_SIZE);

			//  Update checksum, etc, and write the RAM copy of SYSTEM and APPLICATION attributes to SPI Flash
			WriteFlashParameterTable();
		}
	}

	return eRetVal;
}


// Calculate the checksum of the RAM System Parameter table
static UINT16 ChecksumSystemParameterTable(UINT8 *table)
{
    INT16 sum = -1;
    UINT16 i;
    UINT16* ptr = (UINT16*)table;

    /* Regenerate the checksum of the entire SYSTEM and APPLICATION attribute table */
    for(i = 0; i < FLASH_SYS_PARAMETERS_TABLE_SIZE / sizeof(UINT16); i++, ptr++)
	{
        sum += *ptr;
    }

    return sum;
}


#ifdef SUPPORT_RESET_BUTTON
	/* Check for the user holding a factory reset button to wipe the flash parameters */
	static BOOL FlashTableReset(void)
	{
		if(getGpio(FACTORY_RESET_PIN) > 0) 
		{
			RTOS_Fixed_Delay_S(3);
			if(getGpio(FACTORY_RESET_PIN) > 0)
				return TRUE;
		}
		return FALSE;
	}
#endif	//  SUPPORT_RESET_BUTTON

	
// *************************************************************************************************
//							Write System Parameter Table to Flash Storage
// *************************************************************************************************

// update the RAM copy of the System Parameter table and write to external SPI Flash
void WriteFlashParameterTable(void)
{

	if (gbSPIFlashPresent IS FALSE)					// SPI flash is either not present or not functional
	{
		RuntimeError(SYSTEM_ERROR_FLASH_NOT_PRESENT);
		return;
	}

	// bump structure update counter
	++(ptrRAM_SystemParameters->unUpdateCtr);

	/* Regenerate the magic word (changes with change of Attribute structure size) */
	ptrRAM_SystemParameters->unMagic = DEFAULT_SYS_FLASH_MAGIC_NUMBER;

	/* Regenerate the checksum (changes with any change of Attribute values) */
	ptrRAM_SystemParameters->unChecksum = 0;
	ptrRAM_SystemParameters->unChecksum = -(INT16)ChecksumSystemParameterTable((UINT8*)ptrRAM_SystemParameters);

	/* Erase the 4K sector (assumes flash table size is < sector size)*/
	SST25SectorErase((DWORD)SPI_FLASH_TABLE_ADDRESS);

	/* Write the System Parameters table */
	IGNORE_RETURN_VALUE SST25WriteArray((DWORD)SPI_FLASH_TABLE_ADDRESS, (UINT8*)ptrRAM_SystemParameters, (WORD)FLASH_SYS_PARAMETERS_TABLE_SIZE);

	// ==> should have a return value

}

	
// *************************************************************************************************
//							Read System Parameter Table from SPI Flash Storage to RAM
// *************************************************************************************************
	
static void ReadFlashParameterTable()
{
	if (gbSPIFlashPresent IS FALSE)					// SPI flash is either not present or not functional
	{
		RuntimeError(SYSTEM_ERROR_FLASH_NOT_PRESENT);
		return;
	}

	/* Write the System Parameters table */
	SST25ReadArray((DWORD)SPI_FLASH_TABLE_ADDRESS, (UINT8*)ptrRAM_SystemParameters, (WORD)FLASH_SYS_PARAMETERS_TABLE_SIZE);

	// ==> needs a return value

}	


// *************************************************************************************************
//						Load and Initialize RTCC RAM Parameter
// *************************************************************************************************


enum tagMemoryInit InitRTCCRAMParameterTable(void)
{
	enum tagMemoryInit eRetVal = MEMORY_INITIALIZED;
	UINT16	nCheckSum;

	#ifdef USE_DS3232_RTCC
		if (ReadRTCCRAMParameterTables() IS_FALSE)		// read entire parameter table from RTCC NV RAM to MCU RAM
														// read saved orientation from RTCC NV RAM to MCU RAM
		{
			eRetVal = MEMORY_NOT_PRESENT;
		}
	#endif

	// calculate checksum of System Parameter table
    nCheckSum = ChecksumRTCCRAMParameterTable((UINT8 *)ptrRTCC_RAM_AppParameters);

	// check for valid Attribute table
	// if	checksum is not correct (not 0)
	//		'magic' number in attributes copied from SPI flash does not match 'magic' number for this build
	//		structure end marker has moved relative to expected location for this build (which should have changed the 'magic' number
	// copy the defaultSystemParameters table to RAM, update the checksum and 'magic' number, and write out to SPI flash
	if ((nCheckSum) OR (ptrRTCC_RAM_AppParameters->unMagic IS_NOT DEFAULT_APP_RAM_MAGIC_NUMBER) OR (ptrRTCC_RAM_AppParameters->unEndMarker IS_NOT DEFAULT_TABLE_END_MARKER))
	{
		eRetVal = MEMORY_CHECKSUM_ERROR;
		RuntimeError(SYSTEM_ERROR_RAM_PARAMETER_TABLE);

		// Replace the current RAM copy of the system parameters from default table
		memcpy((UINT8*)ptrRTCC_RAM_AppParameters, (UINT8*)&defaultRTCC_RAM_AppParameters, RTCC_RAM_APP_PARAMETERS_TABLE_SIZE);

		#ifdef USE_DS3232_RTCC
			//  Update checksum, etc, and write the RAM copy of SYSTEM and APPLICATION attributes to SPI Flash
			IGNORE_RETURN_VALUE WriteRTCCRAMParameterTable();
		#endif
    }

	return eRetVal;				// return value checksum should account for return value from WriteRTCCRAMParameterTable()
}

// Calculate the checksum of the MCU RAM copy of the RTCC NV RAM Parameter table
// Operates on RAM only, no access to RTCC
static UINT16 ChecksumRTCCRAMParameterTable(UINT8 *table)
{
    INT16 sum = -1;
    UINT16 i;
    UINT16* ptr = (UINT16*)table;

    /* Regenerate the checksum of the entire SYSTEM and APPLICATION attribute table */
    for(i = 0; i < RTCC_RAM_APP_PARAMETERS_TABLE_SIZE / sizeof(UINT16); i++, ptr++)
	{
        sum += *ptr;
    }

    return sum;
}



// *************************************************************************************************
//							Write System Parameter Table to RTCC RAM
// *************************************************************************************************

// update the RAM copy of the System Parameter table and write RTCC RAM
// currently ONLY called on each SPA calculation
BOOL WriteRTCCRAMParameterTable(void)
{
	BOOL bRetVal = TRUE;

	// bump structure update counter
	++(ptrRTCC_RAM_AppParameters->unUpdateCtr);

	/* Regenerate the magic word (changes with change of Attribute structure size) */
	ptrRTCC_RAM_AppParameters->unMagic = DEFAULT_APP_RAM_MAGIC_NUMBER;

	// copy the Current Orientation to MCU RAM copy of RTCC NV RAM
//	ptrRTCC_RAM_AppParameters->lLastAzimuth = CurrentPosition_Read(MOTOR_AZIMUTH);
//	ptrRTCC_RAM_AppParameters->lLastElevation = CurrentPosition_Read(MOTOR_ELEVATION);

	/* Regenerate the checksum (changes with any change of Parameter values) */
	ptrRTCC_RAM_AppParameters->unChecksum = 0;
	ptrRTCC_RAM_AppParameters->unChecksum = -(INT16)ChecksumRTCCRAMParameterTable((UINT8*)ptrRTCC_RAM_AppParameters);

	#ifdef USE_DS3232_RTCC
		// write the MCU RAM copy of the RTCC NV RAM to RTCC
		bRetVal = WriteRTCCRAMArray(0, (UINT8*)ptrRTCC_RAM_AppParameters, RTCC_RAM_APP_PARAMETERS_TABLE_SIZE);
	#endif

	return(bRetVal);

}

// *************************************************************************************************
//							Update System Parameter Table to RTCC RAM
// *************************************************************************************************

// RAM copy of Current Orientation is updated by MotionSensor.c: MotionSensor_Tick() on each MSI Tick
// write the MCU RAM copy of the Current Orientation to RTCC RAM
BOOL UpdateRTCCRAMOrientation(void)
{
	BOOL bRetVal;

	UINT8 NVRAM_offset = CURRENT_ORIENTATION_OFFSET;					// structure is in RTCC RAM, after RTCC_RAM_AppParameters table
	UINT8 *ptrMCURAM = ((UINT8*)ptrRTCC_RAM_MechanicalOrientation);			// pointer to Orientation structure in MCU RAM
	UINT8 size = MECHANICAL_ORIENTATION_TICKS_SIZE;						// size of structure in either MCU RAM or RTCC RAM

//	bRetVal = WriteRTCCRAMArray(offsetof(RTCC_RAM_APP_PARAMETERS, lLastAzimuth), (UINT8*)ptrRTCC_RAM_AppParameters->lLastAzimuth, MECHANICAL_ORIENTATION_TICKS_SIZE);

	#ifdef USE_DS3232_RTCC
		bRetVal = WriteRTCCRAMArray(NVRAM_offset, ptrMCURAM, size);
	#endif

	return(bRetVal);

}



// *************************************************************************************************
//							Read System Parameter Table from  RTCC RAM to MCU RAM
// *************************************************************************************************

BOOL ReadRTCCRAMParameterTables()
{
	BOOL bRetVal = TRUE;

	#ifdef USE_DS3232_RTCC
		bRetVal = ReadRTCCRAMArray(SMARTTRAK_APP_RAM_TABLE_OFFSET, (UINT8*)ptrRTCC_RAM_AppParameters, RTCC_RAM_APP_PARAMETERS_TABLE_SIZE);
		if (bRetVal IS_TRUE)
		{
			bRetVal = ReadRTCCRAMArray(CURRENT_ORIENTATION_OFFSET, (UINT8*)ptrRTCC_RAM_MechanicalOrientation, MECHANICAL_ORIENTATION_TICKS_SIZE);
		}
	#endif

	return bRetVal;
}


// *************************************************************************************************
//							Clear Current Orientation
// *************************************************************************************************
BOOL ClearRTCCRAMOrientation(void)
{
	BOOL bRetVal = TRUE;

	// clear MCU RAM copy of current orientation
	ptrRTCC_RAM_MechanicalOrientation->lLastAzimuth = 0L;
	ptrRTCC_RAM_MechanicalOrientation->lLastElevation = 0L;

	#ifdef USE_DS3232_RTCC
		// update RTCC NV RAM copy of current orientation
		bRetVal = (UpdateRTCCRAMOrientation());
	#endif

	return bRetVal;

}


// *************************************************************************************************
//							Clear ALL RTCC RAM
// *************************************************************************************************
BOOL ClearRTCCRAM(void)
{
	BOOL bRetVal = TRUE;

	#ifdef USE_DS3232_RTCC
		// clear ALL RTCC RAM
		bRetVal = ClearRTCCRAMArray(0x00, DS3232_SRAM_LEN);
	#endif

	return bRetVal;

}

// end of SystemParameters.c

