// *************************************************************************************************
//										D S 3 2 3 2 . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	DS3232 RTCC Function declarations
// *************************************************************************************************

#ifndef DS3232_H
	#define DS3232_H
#endif

typedef struct
{
	UINT8	cSeconds;
	UINT8	cMinutes;
	UINT8	cHours;
	BOOL	bAM_PM;
	BOOL	b12_24;
	UINT8	cDay;
	UINT8	cDate;
	UINT8	cMonth;
	BOOL	bCentury;
	UINT16	nYear;				// complete year number

} RTCC_DATE_TIME, *PTR_RTCC_DATE_TIME;

BOOL ReadRTCCDateTime(PTR_RTCC_DATE_TIME ptrDateTime);
BOOL WriteRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen);
BOOL ClearRTCCRAMArray(UINT8 cDataAddress, UINT8 cDataLen);
BOOL ReadRTCCRAMArray(UINT8 cDataAddress, UINT8 *ptrData, UINT8 cDataLen);

BOOL WriteRTCCRegister(UINT8 cRegAddress, UINT8 bData);
BOOL ReadRTCCRegister(UINT8 cRegAddress, UINT8 *ptrData);


// *************************************
//		Register Addresses
// *************************************
#define	DS3232_REGISTERS_START		0x00
#define	DS3232_REGISTERS_LEN		0x14

#define	DS3232_SRAM_START			0x14
#define	DS3232_SRAM_LEN				235		// (0xFF - 0x14)

#define	DS3232_I2C_ADDR				0x68	// formatted assuming it will be shifted one bit left

#define DS3232_REG_SECONDS			0x00
#define DS3232_REG_MINUTES			0x01
#define DS3232_REG_HOURS			0x02
#define DS3232_REG_AMPM				0x02
#define DS3232_REG_DAY				0x03
#define DS3232_REG_DATE				0x04
#define DS3232_REG_MONTH			0x05
#define DS3232_REG_CENTURY			0x05
#define DS3232_REG_YEAR				0x06
#define DS3232_REG_ALARM1			0x07	/* Alarm 1 BASE */
#define DS3232_REG_ALARM2			0x0B	/* Alarm 2 BASE */
#define DS3232_REG_CR				0x0E	/* Control register */
#	define DS3232_REG_CR_nEOSC        0x80
#       define DS3232_REG_CR_INTCN       0x04
#       define DS3232_REG_CR_A2IE        0x02
#       define DS3232_REG_CR_A1IE        0x01

#define DS3232_REG_SR	0x0F	/* control/status register */
#	define DS3232_REG_SR_OSF   0x80
#       define DS3232_REG_SR_BSY   0x04
#       define DS3232_REG_SR_A2F   0x02
#       define DS3232_REG_SR_A1F   0x01

#define DS3232_SECONDS_MASK			0x7F	// mask for BCD seconds
#define DS3232_MINUTES_MASK			0x7F	// mask for BCD minutes
#define DS3232_12_HOURS_MASK		0x1F	// mask for BCD 12 hours
#define DS3232_24_HOURS_MASK		0x3F	// mask for BCD 24 hours
#define DS3232_12_NOT24_HOURS_MASK	0x40	// mask for 12 /24 hours flag in hours register
#define DS3232_AM_NOTPM_MASK		0x20	// mask for 12 /24 hours flag in hours register
#define DS3232_DAY_MASK				0x07	// mask for BCD day (OK, so it is NOT really BCD)
#define DS3232_DATE_MASK			0x3F	// mask for BCD date
#define DS3232_MONTH_MASK			0x3F	// mask for BCD date
#define DS3232_CENTURY_MASK			0x80	// mask for Century bit in Month register
#define DS3232_YEAR_MASK			0xFF	// mask for BCD year


// *************************************
//		Limit Values
// *************************************
#define	DS3232_MIN_SECONDS			0
#define	DS3232_MAX_SECONDS			59

#define	DS3232_MIN_MINUTES			0
#define	DS3232_MAX_MINUTES			59

#define	DS3232_MIN_HOURS_24			0
#define	DS3232_MAX_HOURS_24			23

#define	DS3232_MIN_HOURS_12			1
#define	DS3232_MAX_HOURS_12			12

#define	DS3232_MIN_DAY				1
#define	DS3232_MAX_DAY				7

#define	DS3232_MIN_DATE				1
#define	DS3232_MAX_DATE				31

#define	DS3232_MIN_MONTH			1
#define	DS3232_MAX_MONTH			12

#define	DS3232_MIN_YEAR				12
#define	DS3232_MAX_YEAR				99



#ifndef DEFINE_GLOBALS
	#define	DEFINE_EXTERNS
#endif

#if defined (DEFINE_GLOBALS)

	GLOBAL_INIT ARRAY  const char *pstrDayText[] =
		{
		" ???",
		" Sun",
		" Mon",
		" Tue",
		" Wed",
		" Thr",
		" Fri",
		" Sat"
		};

	GLOBAL_INIT ARRAY  const char *pstrMonthText[] =
		{
		" ???",
		" Jan",
		" Feb",
		" Mar",
		" Apr",
		" May",
		" Jun",
		" Jul",
		" Aug",
		" Sep",
		" Oct",
		" Nov",
		" Dec"
		};


#elif defined (DEFINE_EXTERNS)
	GLOBAL	ARRAY  const char *pstrDayText[];
	GLOBAL	ARRAY  const char *pstrMonthText[];
#endif


// end of DS3232.h




