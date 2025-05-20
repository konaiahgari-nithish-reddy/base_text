



#include "ftoa.h"
#include <stdlib.h>

char *errStr = "ftoa ERR";

char *_ftoa( float f, char cNumPlaces, int *status);			// <sek> added number of places

typedef union
    {
    long L;
    float F;
    } LF_t;

char *ftoa( float f, int *status)							// <sek> shell function
{
	return(_ftoa(f, 4, status));							// limit to 4 places because smallest input unit is 0.0104 (1 tick as degrees)
}

char *ftoa2( float f, int *status)							// <sek> shell function
{
	return(_ftoa(f, 2, status));							// limit to 1 places because smallest input unit is 0.044 (1 count of 4094 as degrees, inclinometer)
}


char *_ftoa( float f, char cNumPlaces, int *status)			// <sek> added number of places
    {
    long mantissa, int_part, frac_part;
    short exp2;
    LF_t x;
    char *p;
    static char outbuf[15];

    *status = 0;

    if ( f == 0.0 )
        {
        outbuf[0] = '0';
        outbuf[1] = '.';
        outbuf[2] = '0';
        outbuf[3] = 0;
        return outbuf;
        }
    x.F = f;

    exp2 = (unsigned char)(x.L >> 23) - 127;
    mantissa = (x.L & 0xFFFFFF) | 0x800000;
    frac_part = 0;
    int_part = 0;

    if ( exp2 >= 31 )
        {
        *status = _FTOA_TOO_LARGE;
        //return 0;
		return errStr;
        }
    else if ( exp2 < -23 )
        {
        *status = _FTOA_TOO_SMALL;
        //return 0;
		return errStr;
        }
    else if ( exp2 >= 23 )
        int_part = mantissa << (exp2 - 23);
    else if ( exp2 >= 0 )
        {
        int_part = mantissa >> (23 - exp2);
        frac_part = (mantissa << (exp2 + 1)) & 0xFFFFFF;
        }
    else /* if (exp2 < 0) */
        frac_part = (mantissa & 0xFFFFFF) >> -(exp2 + 1);

    p = outbuf;

    if ( x.L < 0 )
        *p++ = '-';

    if ( int_part == 0 )
        *p++ = '0';
    else
        {
        ltoa(p, int_part, 10);

        while ( *p )
            p++;
        }
    *p++ = '.';

    if ( frac_part == 0 )
        *p++ = '0';
    else
        {
        char m, max;

        max = sizeof(outbuf) - (p - outbuf) - 1;

        if ( max > 7 )
            max = 7;
		if (max > cNumPlaces)			// <sek> limit number of places
			max = cNumPlaces;

        /* print BCD */
        for( m = 0; m < max; m++ )
            {
            /* frac_part *= 10; */
            frac_part = (frac_part << 3) + (frac_part << 1);

            *p++ = (frac_part >> 24) + '0';
            frac_part &= 0xFFFFFF;
            }
        /* delete ending zeroes */
        for( --p; p[0] == '0' && p[-1] != '.'; --p )
        ;
        ++p;
        }
    *p = 0;

    return outbuf;
    }

//char *ftoa( float f, int *status, char *buffer )
//    {
    //... return buffer;
//    }

//One bug was uncovered here->


//exp2 = (0xFF & (x.L >> 23)) - 127; /* JEB fixed for 16-bit char F2xxx */