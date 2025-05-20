// *************************************************************************************************
//					R T C C . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	RTCC Functions
//
// *************************************************************************************************

//-----------------------------------------------------------------------------
//			#include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>
#include "config.h"				// compile time configuration definitions

#include <string.h>				// for strcpy(), strcat()

#include "gsfstd.h"				// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions

#include "SerialDisplay.h"		// display functions for menus
//#include "MenuFSM.h"
#include "StrConversions.h"		// ASCII string <==> numeric conversions

#include "DS3232.h"				// RTCC register level
#include "RTCC.h"

// return RTCC Date and Time as a formatted string
// Output format: Day Date Month Year Hours:Minutes:Seconds

BOOL FormatRTCCDateTime(char *ptrOutputStr, PTR_RTCC_DATE_TIME ptrDateTime)
{
	char strTemp[20];


	strcpy(ptrOutputStr, "\t");

	BYTEtoASCIIstr(ptrDateTime->cHours, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cMinutes, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cSeconds, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, " ");


	BYTEtoASCIIstr(ptrDateTime->cDate, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, " ");

	strcat(ptrOutputStr, pstrMonthText[ptrDateTime->cMonth]);
	strcat(ptrOutputStr, " ");

	WORDtoASCIIstr(ptrDateTime->nYear, WORD_WIDTH, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, " ");


	strcat(ptrOutputStr, pstrDayText[ptrDateTime->cDay]);
	strcat(ptrOutputStr, " ");

	return TRUE;

}

BOOL ADDRTCCDateTime(char *ptrOutputStr, PTR_RTCC_DATE_TIME ptrDateTime)
{
	char strTemp[20];

	BYTEtoASCIIstr(ptrDateTime->cHours, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cMinutes, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cSeconds, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cDate, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	strcat(ptrOutputStr, pstrMonthText[ptrDateTime->cMonth]);
	strcat(ptrOutputStr, ":");

	WORDtoASCIIstr(ptrDateTime->nYear, WORD_WIDTH, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	strcat(ptrOutputStr, pstrDayText[ptrDateTime->cDay]);


	return TRUE;

}

BOOL ADD2RTCCDateTime(char *ptrOutputStr, PTR_RTCC_DATE_TIME ptrDateTime)
{
	char strTemp[20];

	BYTEtoASCIIstr(ptrDateTime->cHours, strTemp);
	strcpy(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cMinutes, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cSeconds, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cDate, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(ptrDateTime->cMonth, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	WORDtoASCIIstr(ptrDateTime->nYear, WORD_WIDTH, strTemp);
	strcat(ptrOutputStr, strTemp);
	//strcat(ptrOutputStr, ":");

	//strcat(ptrOutputStr, pstrDayText[ptrDateTime->cDay]);


	return TRUE;

}


BOOL ADDBUILDDateTime(char *ptrOutputStr)
{
	char strTemp[20];

	BYTEtoASCIIstr(BUILD_HOUR, strTemp);
	strcpy(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(BUILD_MIN, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(BUILD_SEC, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(BUILD_DAY, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	BYTEtoASCIIstr(BUILD_MONTH, strTemp);
	strcat(ptrOutputStr, strTemp);
	strcat(ptrOutputStr, ":");

	WORDtoASCIIstr(BUILD_YEAR, WORD_WIDTH, strTemp);
	strcat(ptrOutputStr, strTemp);
	//strcat(ptrOutputStr, ":");

	//strcat(ptrOutputStr, pstrDayText[ptrDateTime->cDay]);


	return TRUE;

}
int dater(int x)
{ int y=0;
switch(x)
{
case 1: y=0; break;
case 2: y=31; break;
case 3: y=59; break;
case 4: y=90; break;
case 5: y=120;break;
case 6: y=151; break;
case 7: y=181; break;
case 8: y=212; break;
case 9: y=243; break;
case 10:y=273; break;
case 11:y=304; break;
case 12:y=334; break;
}
return(y);
}

int GETDIFF(char *ptrOutputStr,PTR_RTCC_DATE_TIME ptrDateTime)
{
	char strTemp[20],strTemp1[20],strTemp2[20];
	int bdate,rdate,diff,dd1=0,dd2=0,i=0,stop_bit=0;

	WORDtoASCIIstr(ptrDateTime->nYear, WORD_WIDTH, strTemp);
	strcpy(strTemp1, strTemp);
	BYTEtoASCIIstr(ptrDateTime->cMonth, strTemp);
	strcat(strTemp1, strTemp);
	BYTEtoASCIIstr(ptrDateTime->cDate, strTemp);
	strcat(strTemp1, strTemp);
	strcpy(ptrOutputStr, strTemp1);
	strcat(ptrOutputStr, ":");


	WORDtoASCIIstr(BUILD_YEAR, WORD_WIDTH, strTemp);
	strcpy(strTemp2, strTemp);
	BYTEtoASCIIstr(BUILD_MONTH, strTemp);
	strcat(strTemp2, strTemp);
	BYTEtoASCIIstr(BUILD_DAY, strTemp);
	strcat(strTemp2, strTemp);
	bdate = atoi(strTemp2);
	strcat(ptrOutputStr, strTemp2);

	strcat(ptrOutputStr, ":");

	rdate = atoi(strTemp1);
	diff = rdate-bdate;

	dd1=dater(ptrDateTime->cMonth);
	for(i=BUILD_YEAR;i<=(ptrDateTime->nYear);i++)
	{
		if(i%4==0)
			dd1+=1;
	}
	dd1=dd1+ptrDateTime->cDate+(ptrDateTime->nYear-BUILD_YEAR)*365;

	dd2=0;
	for(i=BUILD_YEAR;i<BUILD_YEAR;i++)
	{
		if(i%4==0)
			dd2+=1;
	}
	dd2=dater(BUILD_MONTH)+dd2+BUILD_DAY;

	diff = dd1-dd2;
	sprintf(strTemp,"%d",diff);
	strcat(ptrOutputStr, strTemp);

	if(ptrRAM_SystemParameters->fSingle_stop_days == 0)
		stop_bit = 0;
	else if(diff > ptrRAM_SystemParameters->fSingle_stop_days)
		stop_bit = 1;

	return stop_bit;

}

// end of RTCC.c
