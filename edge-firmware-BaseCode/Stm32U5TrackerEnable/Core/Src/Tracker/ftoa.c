#include "ftoa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char *errStr = "ftoa ERR";

char *_ftoa( float f, char cNumPlaces, int *status);			// <sek> added number of places

typedef union
{
	long L;
	float F;
} LF_t;

//		void Itoa(char *p,int data,int size)
//		{
//			int i,j;
//			char *ptr=NULL;
//			while(data!=0)
//			{
//					*ptr=(data %10) +48;
//						ptr++;
//				data/=10;
//			}
//			for(i=0,j=strlen(ptr)-1;j>=0;i++,j--)
//						p[i]=ptr[j];
//		}


#define BUFSIZE (sizeof(long) * 8 + 1)

char *Ltoa(long N, char *str, int base)
{
	register int i = 2;
	long uarg;
	char *tail, *head = str, buf[BUFSIZE];

	if (36 < base || 2 > base)
		base = 10;                    /* can only use 0-9, A-Z        */
	tail = &buf[BUFSIZE - 1];           /* last character position      */
	*tail-- = '\0';

	if (10 == base && N < 0L)
	{
		*head++ = '-';
		uarg    = -N;
	}
	else  uarg = N;

	if (uarg)
	{
		for (i = 1; uarg; ++i)
		{
			register ldiv_t r;

			r       = ldiv(uarg, base);
			*tail-- = (char)(r.rem + ((9L < r.rem) ?
					('A' - 10L) : '0'));
			uarg    = r.quot;
		}
	}
	else  *tail-- = '0';

	memcpy(head, ++tail, i);
	return str;
}


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

	if ( f == (float)0.0 )
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
		p=Ltoa(int_part,p,10);
		//  _ltoa((long)int_part,(char *)p,(int)10);

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
