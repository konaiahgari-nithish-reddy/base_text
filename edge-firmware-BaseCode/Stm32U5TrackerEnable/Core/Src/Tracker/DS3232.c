// *************************************************************************************************
//										D S 3 2 3 2 . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	DS3232 RTCC Functions
//
// *************************************************************************************************

// These functions are NOT register level. They call into the PIC32 Peripheral library to
// handle most register access, especially initialization

// NOTE: Write Array functions now use a single call to write_i2C_device() to write multiple bytes.
//		 Read Array functions still use one call to read_i2c_device() per byte, because of previously observed problems
//				this MAY be changed, pending additional testing.

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>
#include "config.h"				// compile time configuration definitions

#include "stm32u5xx_hal.h"

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions
#include "StrConversions.h"

#include "DS3232.h"

// ****************************************************************************
// ****************************************************************************

// ****************************************************************************
// ****************************************************************************

extern RTC_HandleTypeDef hrtc;

// return RTCC Date and Time in a structure of BYTES and INTs, converted from BCD
// NOTE: implementee as one byte per call to read_i2c_device() because of varying conversion code.

BOOL ReadRTCCDateTime(PTR_RTCC_DATE_TIME ptrDateTime)
{
	BOOL bRetval = TRUE;
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetval = FALSE;
		return bRetval;
	}

	ptrDateTime->cHours = sTime.Hours;
	ptrDateTime->cMinutes = sTime.Minutes;
	ptrDateTime->cSeconds = sTime.Seconds;
	ptrDateTime->bAM_PM = sTime.TimeFormat;
	ptrDateTime->b12_24 = RTC_HOURFORMAT_24;

	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetval = FALSE;
		return bRetval;
	}

	ptrDateTime->cDate = sDate.Date;
	ptrDateTime->cMonth = sDate.Month;
	ptrDateTime->cDay = sDate.WeekDay;
	ptrDateTime->nYear = 2000 + sDate.Year;

	return bRetval;
}

/************************************************************************
 * Function: BOOL ReadRTCCRegister(UINT8 cRegAddress, UINT8 *ptrData)
 *
 * Overview: this function reads a single RTCC Register into buffer specified
 *
 * Input: register offset from 0, pointer to the data buffer
 *
 * Output: return TRUE if the operation was successfull
 *
 ************************************************************************/

BOOL ReadRTCCRegister(UINT8 cRegAddress, UINT8 *ptrData)
{
	BOOL bRetVal = TRUE;

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	if (cRegAddress == DS3232_REG_SECONDS)
		*ptrData = sTime.Seconds;
	else if (cRegAddress == DS3232_REG_MINUTES)
		*ptrData = sTime.Minutes;
	else if (cRegAddress == DS3232_REG_HOURS)
		*ptrData = sTime.Hours;
	else if (cRegAddress == DS3232_REG_DAY)
		*ptrData = sDate.WeekDay;
	else if (cRegAddress == DS3232_REG_DATE)
		*ptrData = sDate.Date;
	else if (cRegAddress == DS3232_REG_MONTH)
		*ptrData = sDate.Month;
	else if (cRegAddress == DS3232_REG_YEAR)
		*ptrData =  sDate.Year;
	else if (cRegAddress == DS3232_REG_AMPM)
		*ptrData = sTime.TimeFormat;

	return bRetVal;
}


/************************************************************************
 * Function: BOOL WriteRTCCRegister(UINT8 cRegAddress, UINT8 bData)
 *
 * Overview: this function writes a single RTCC Register
 *
 * Input: register offset from 0, data
 *
 * Output: return TRUE if the operation was successfull
 *
 ************************************************************************/
BOOL WriteRTCCRegister(UINT8 cRegAddress, UINT8 bData)
{

	BOOL bRetVal = TRUE;

	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};

	if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	if (cRegAddress == DS3232_REG_SECONDS)
		sTime.Seconds = bData;
	else if (cRegAddress == DS3232_REG_MINUTES)
		sTime.Minutes = bData;
	else if (cRegAddress == DS3232_REG_HOURS)
		sTime.Hours = bData;
	else if (cRegAddress == DS3232_REG_DAY)
		sDate.WeekDay = bData;
	else if (cRegAddress == DS3232_REG_DATE)
		sDate.Date = bData;
	else if (cRegAddress == DS3232_REG_MONTH)
		sDate.Month = bData;
	else if (cRegAddress == DS3232_REG_YEAR)
		sDate.Year = bData;
	else if (cRegAddress == DS3232_REG_AMPM)
		sTime.TimeFormat = bData;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		bRetVal = FALSE;
		return bRetVal;
	}

	return bRetVal;
}


// end of DS3232.c

