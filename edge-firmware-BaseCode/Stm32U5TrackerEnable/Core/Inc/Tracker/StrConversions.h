// *************************************************************************************************
//										S t r C o n v e r s i o n s . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	string <=> numeric conversion function definitions
//
// *************************************************************************************************
#include"GenericTypeDefs.h"
#include"gsfstd.h"
//-------------------------------------------------------------------------------------------------------
// Definitions
//-------------------------------------------------------------------------------------------------------

// these are replacements for sprintf(), which is WAY too slow
// this collection of format-specific functions should run much faster than a generalized function
// NOTE: we will still need to use sprintf() to handle floating point formatting

void BYTEtoHexASCIIstr(BYTE cData, char *pStr);
void BYTEtoASCIIstr(BYTE cData, char *pStr);
void WORDtoHexASCIIstr(WORD wData, char *pStr);
void WORDtoASCIIstr(WORD wData, BYTE cOutputWidth, char *pStr);
void INT16StoASCIIstr(INT16S nData, BYTE cOutputWidth, char *pStr);
void INT32StoASCIIstr(INT32S lData, BYTE cOutputWidth, char *pStr);
void INT32UtoASCIIstr(INT32U lData, BYTE cOutputWidth, char *pStr);
void INT32UtoHexASCIIstr(INT32U lData, char *pStr);


UINT8 BCDtoBYTE( UINT8 bcd);
UINT8 BYTEtoBCD( UINT8 bByte);
void BCDBytetoASCIIstr(BYTE bBCD, char *pStr);


#define	BYTE_WIDTH			3			// byte as decimal ASCII
#define	BYTE_99_WIDTH		2			// byte as decimal ASCII, limited to 0 to 99
#define	BYTE_9_WIDTH		1			// byte as decimal ASCII, limited to 0 to 9
#define INT8S_WIDTH			3
#define	TEN_BIT_WORD_WIDTH	4			// 10 bit value as decimal ASCII (ADC values)
#define	WORD_WIDTH			5			// unsigned 16 bit value
#define	INT16S_WIDTH		6			// width must account for the number AND sign
#define	INT18S_WIDTH		7			// width must account for the number AND sign
#define	INT18U_WIDTH		6			// width must account for the number
#define	INT32U_WIDTH		10			// unsigned 32 bit value
#define	INT32S_WIDTH		11			// signed 32 bit value, width must account for the number AND sign

//#define	HEX_BYTE_WIDTH		2			// byte as HEX ASCII
//#define	HEX_WORD_WIDTH		4

// end of StrConversions.h
