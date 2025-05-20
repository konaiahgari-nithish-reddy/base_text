// *************************************************************************************************
//								S t r C o n v e r s i o n s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	Debug Runtime Error Handler functions
// *************************************************************************************************

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

// processor include file
//lint -e765					error 765: (Info -- external function could be made static)
//lint -e14						error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>				// Microchip PIC32 peripheral library main header
//lint +e14


#include <string.h>				// Microchip string functions

#include "gsfstd.h"				// gsf standard #defines, use <> so lint will treat this as a library file
#include "StrConversions.h"			


PRIVATE char HextoASCII(BYTE data);


// *****************************************************************************
//				s p r i n t ( ), s c a n f ( )   R e p l a c e m e n t s
// *****************************************************************************

// NOTE: all of these functions are written to work with RAM based data, both input and output

// *****************************************************************************
//							Data to ASCII
// *****************************************************************************

// this array (a string, really) is used to convert a single nibl of hex data to its equivalent ASCII value
FILE_GLOBAL ARRAY char fgcHexToAscii[] = "0123456789ABCDEF";
//lint -e585  error 585: (Warning -- The sequence (???) is not a valid Trigraph sequence)
FILE_GLOBAL ARRAY char fgcBCDToAscii[] = "0123456789??????";
//lint +e585

PRIVATE char HextoASCII(BYTE data)
{
	if (data <= 0x0F)
		{
		return(fgcHexToAscii[data]);
		}
	else
		{
		return('?');		// meaningless placekeeper
		}
}

// unsigned 8 bit to Decimal ASCII string
/* not used anywhere
char DectoASCII(BYTE data)
{
	if (data <= 0x09)
		{
		return(fgcHexToAscii[data]);
		}
	else
		{
		return('?');		// meaningless placekeeper
		}
}
*/

void BYTEtoASCIIstr(BYTE cData, char *pStr)
{
	// NOTE: output width of 1 will not work with implementation of WORDtoASCIIstr
	if (cData <= 99)
	{
		WORDtoASCIIstr((WORD) cData, BYTE_99_WIDTH, pStr);
	}
	else
	{
		WORDtoASCIIstr((WORD) cData, BYTE_WIDTH, pStr);
	}

}

// unsigned 8 bit to HEX ASCII string
void BYTEtoHexASCIIstr(BYTE cData, char *pStr)
	{
	// convert hi nibl
	*pStr = HextoASCII((cData >> 4) & 0x0F);
	++pStr;

	// convert low nibl
	*pStr = HextoASCII(cData & 0x0F);
	++pStr;

	// add terminator
	*pStr = '\0';
	}

// unsigned 8 bit to HEX ASCII string
void WORDtoHexASCIIstr(WORD wData, char *pStr)
	{
	// convert MSB hi nibl
	*pStr = HextoASCII((wData >> 12) & 0x000F);
	++pStr;

	// convert MSB low nibl
	*pStr = HextoASCII((wData >> 8) & 0x000F);
	++pStr;

	// convert LSB hi nibl
	*pStr = HextoASCII((wData >> 4) & 0x000F);
	++pStr;

	// convert LSB low nibl
	*pStr = HextoASCII(wData & 0x000F);
	++pStr;

	// add terminator
	*pStr = '\0';
	}



// UNsigned 16 bit word to Decimal ASCII string
void WORDtoASCIIstr(WORD wData, BYTE cOutputWidth, char *pStr)
	{
	LOCAL ARRAY char strTemp[10];

	// maximum output width is 5 characters, for decimal equivalent of 16 bits
    BYTE	i = cOutputWidth - 1;			// output index, start at LSD end, leaving one character for the terminator
	BYTE	cOutputCtr = 0;					// count of output characters, to avoid comparing an index to 0

	// generate ASCII digits from LSD to MSD
    do 
		{
		// divide by 10, and convert the REMAINDER into a digit, moving to the LEFT
		strTemp[i--] = wData % 10 + '0';

		++cOutputCtr;						// bump count of output characters

		// divide by 10 to strip away the data just converted
		wData /= 10;
		}
	while ((wData > (WORD)0) AND (cOutputCtr IS_NOT cOutputWidth));	// while we still have something left to convert, and have not used the full output width


	// if we are not out of string, move LEFT and fill the rest with spaces
	while (cOutputCtr IS_NOT cOutputWidth)	// while we have not used the full output width
		{
		strTemp[i] = '0';					// pad with SPACE
		--i;								// bump index LEFT
		++cOutputCtr;						// bump count of output characters
		}

	// add a string terminator
    strTemp[cOutputWidth] = '\0';

	strcpy(pStr, strTemp);
	}


// Signed 16 bit word to Decimal ASCII string
void INT16StoASCIIstr(INT16S nData, BYTE cOutputWidth, char *pStr)
	{
	LOCAL ARRAY char strTemp[10];

	// maximum output width is 5 characters plus sign, for decimal equivalent of 16 bits
    INT16S	nDataCopy;						// copy of original data, to keep track of the sign
    BYTE	i = cOutputWidth - 1;			// output index, start at LSD end, leaving one character for the terminator
	BYTE	cOutputCtr = 0;					// count of output characters, to avoid comparing an index to 0

	nDataCopy = nData;						// keep a copy of the original data value, to keep track of the sign...
    if (nData < 0)							// if nData is negative, we need to work with the absolute value
		{
        nData = -nData;						// make nData positive
		}

	// generate ASCII digits from LSD to MSD
    do 
		{
		// divide by 10, and convert the REMAINDER into a digit, moving to the LEFT
		strTemp[i--] = nData % 10 + '0';

		++cOutputCtr;						// bump count of output characters

		// divide by 10 to strip away the data just converted
		nData /= 10;
		}
	while ((nData > 0) AND (cOutputCtr < cOutputWidth));	// while we still have something left to convert, and have not used the full output width

	// now restore the original sign
    if (nDataCopy < 0)
		{
        strTemp[i] = '-';					// sign character for negative
		}
	else
		{
        strTemp[i] = ' ';					// just a space for positive
		}
	--i;									// bump output index
	++cOutputCtr;							// bump count of output characters to account for sign or space


	// if we are not out of string, move LEFT and fill the rest with spaces
	while (cOutputCtr IS_NOT cOutputWidth)	// while we have not used the full output width
		{
		strTemp[i] = ' ';					// pad with SPACE
		--i;								// bump index LEFT
		++cOutputCtr;						// bump count of output characters
		}

	// add a string terminator
    strTemp[cOutputWidth] = '\0';

	strcpy(pStr, strTemp);
	}


// Signed 32 bit word to Decimal ASCII string
void INT32StoASCIIstr(INT32 lData, BYTE cOutputWidth, char *pStr)
	{
	LOCAL ARRAY char strTemp[12];

	// maximum output width is 10 characters plus sign, for decimal equivalent of 32 bits
    INT32	lDataCopy;						// copy of original data, to keep track of the sign
    BYTE	i = cOutputWidth - 1;			// output index, start at LSD end, leaving one character for the terminator
	BYTE	cOutputCtr = 0;					// count of output characters, to avoid comparing an index to 0

	lDataCopy = lData;						// keep a copy of the original data value, to keep track of the sign...
    if (lData < 0)							// if nData is negative, we need to work with the absolute value
		{
        lData = -lData;						// make nData positive
		}

	// generate ASCII digits from LSD to MSD
    do
		{
		// divide by 10, and convert the REMAINDER into a digit, moving to the LEFT
		strTemp[i--] = lData % 10L + (INT32)'0';

		++cOutputCtr;						// bump count of output characters

		// divide by 10 to strip away the data just converted
		lData /= 10L;
		}
	while ((lData > 0) AND (cOutputCtr < cOutputWidth));	// while we still have something left to convert, and have not used the full output width

	// now restore the original sign
    if (lDataCopy < 0)
		{
        strTemp[i] = '-';					// sign character for negative
		}
	else
		{
        strTemp[i] = ' ';					// just a space for positive
		}
	--i;									// bump output index
	++cOutputCtr;							// bump count of output characters to account for sign or space


	// if we are not out of string, move LEFT and fill the rest with spaces
	while (cOutputCtr IS_NOT cOutputWidth)	// while we have not used the full output width
		{
		strTemp[i] = ' ';					// pad with SPACE
		--i;								// bump index LEFT
		++cOutputCtr;						// bump count of output characters
		}

	// add a string terminator
    strTemp[cOutputWidth] = '\0';

	strcpy(pStr, strTemp);
	}


// UNsigned 32 bit word to Decimal ASCII string
void INT32UtoASCIIstr(INT32U lData, BYTE cOutputWidth, char *pStr)
	{
	LOCAL ARRAY char strTemp[12];

	// maximum output width is 10 characters, for decimal equivalent of 32 bits
    BYTE	i = cOutputWidth - 1;			// output index, start at LSD end, leaving one character for the terminator
	BYTE	cOutputCtr = 0;					// count of output characters, to avoid comparing an index to 0

	// generate ASCII digits from LSD to MSD
    do 
		{
		// divide by 10, and convert the REMAINDER into a digit, moving to the LEFT
		strTemp[i--] = lData % 10L + '0';

		++cOutputCtr;						// bump count of output characters

		// divide by 10 to strip away the data just converted
		lData /= 10L;
		}
	while ((lData > (INT32U)0) AND (cOutputCtr IS_NOT cOutputWidth));	// while we still have something left to convert, and have not used the full output width


	// if we are not out of string, move LEFT and fill the rest with spaces
	while (cOutputCtr IS_NOT cOutputWidth)	// while we have not used the full output width
		{
		strTemp[i] = ' ';					// pad with SPACE
		--i;								// bump index LEFT
		++cOutputCtr;						// bump count of output characters
		}

	// add a string terminator
    strTemp[cOutputWidth] = '\0';

	strcpy(pStr, strTemp);
	}

// unsigned 32 bit word to Hex ASCII string
void INT32UtoHexASCIIstr(INT32U lData, char *pStr)
	{
	// convert MSB hi nibl
	*pStr = HextoASCII((BYTE)((lData >> 12) & 0x000F));
	++pStr;

	// convert MSB low nibl
	*pStr = HextoASCII((BYTE)((lData >> 8) & 0x000F));
	++pStr;

	// convert LSB hi nibl
	*pStr = HextoASCII((BYTE)((lData >> 4) & 0x000F));
	++pStr;

	// convert LSB low nibl
	*pStr = HextoASCII((BYTE)(lData & 0x000F));
	++pStr;

	// add terminator
	*pStr = '\0';
	}


// *****************************************************************************
//						BCD to Data Functions
// *****************************************************************************

#ifdef NOT_USED
// Take a byte (hexval) and return the ASCII character for the highest 4 bits.
UINT8 hex2ascii_h( UINT8 hexval )
    {
    UINT8 tmp;
    tmp = (hexval >> 4) + '0';

    if ( tmp > 0x39 )
        {
        tmp += 7;
        }
    return tmp;
    }


// Take a byte (hexval) and return the ASCII character for the lowest 4 bits.
UINT8 hex2ascii_l( UINT8 hexval )
    {
    UINT8 tmp;
    tmp = (hexval & 0x0F) + '0';

    if ( tmp > 0x39 )
        {
        tmp += 7;
        }
    return tmp;
    }

UINT8 bcd_to_hex( UINT8 bcd )
    {

    UINT8 msn = (bcd >> 4);
    return ((msn * 10) + (bcd & 0x0F));
    }
#endif

// BCD is formatted as two nibls, each representing a decimal digit
UINT8 BCDtoBYTE( UINT8 bcd )
{
    UINT8 msn = (bcd >> 4);					// get MSD

    return ((msn * 10) + (bcd & 0x0F));		// convert to BYTE
}

UINT8 BYTEtoBCD( UINT8 bByte )
{
	UINT8 msd = bByte / 10;
	UINT8 lsd = bByte - (msd * 10);

	return((msd * 16) + lsd);
}


// BCD byte to ASCII decimal string
void BCDBytetoASCIIstr(BYTE bBCD, char *pStr)
{
	// convert hi nibl
	*pStr = fgcBCDToAscii[(bBCD >> 4) & 0x0F];
	++pStr;

	// convert low nibl
	*pStr = fgcBCDToAscii[bBCD & 0x0F];
	++pStr;

	// add terminator
	*pStr = '\0';
}


// *****************************************************************************
//						ASCII to Data Functions
// *****************************************************************************

#ifdef NOTDEF
	BYTE ASCIItoHex(char cData)
	{
		BYTE cRetVal;

		if ((cData >= 'a') AND (cData <= 'f'))
			{
			cRetVal = (BYTE)(10 + (cData - 'a'));			//a - f
			}
		else if ((cData >= 'A') AND (cData <= 'F'))
			{
			cRetVal = (BYTE)(10 + (cData - 'A'));			//A - F
			}
		else if ((cData >= '0') AND (cData <= '9'))
			{
			cRetVal = (BYTE)(cData - 0x30);	// 0 - 9
			}
		else
			{
			return 0;					// not a valid hex character
			}
		
		return(cRetVal);
	}
#endif


// end of MenuFSM.c
