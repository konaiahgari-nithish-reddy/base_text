/*
 * Debug.c
 *
 *  Created on: 01-Sep-2023
 *      Author: Ravi's PC
 */

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"				// gsf standard #defines

#include "Debug.h"
#include "SmartTrak.h"			// Project wide definitions


enum tagDebugErrors
{
	DEBUG_ERROR_NONE = DEBUG_ERROR_BASE,// 0
	DEBUG_UNEXPECTED_TICK,				// 1 unexpected timer tick event
	DEBUG_ERROR_UNEXPECTED_EVENT,		// 2 unexpected event
	DEBUG_ERROR_INVALID_STATE,			// 3 not a valid state
	DEBUG_ERROR_INVALID_SUBSTATE,		// 4 not a valid state
	DEBUG_ERROR_INVALID_LED_SEQUENCE,	// 5 not a LED sequence number
	DEBUG_ERROR_STACK_OVERFLOW,			// 6 stack overflow

	DEBUG_UNPROCESSED_EVENT = DEBUG_ERROR_BASE + 0x0F
};

unsigned int	pgwRuntimeErrorBuffer[ERROR_BUFFER_SIZE];
unsigned int		pgwErrorIndex;

//-------------------------------------------------------------------------------------------------------
//  RuntimeError
//
//  Input: 	    error value
//  Output:
//  Desc:		writes all runtime error codes to a ring buffer to simplify debugging
//
//-------------------------------------------------------------------------------------------------------

void _RuntimeError(WORD wErrorCode)
{

    // write error code to ring buffer
    pgwRuntimeErrorBuffer[pgwErrorIndex] = wErrorCode;

	// check for end of the buffer
    if (pgwErrorIndex == ERROR_BUFFER_MAX_INDEX)
		{
		// we just wrote to the end of the buffer, so start over
        pgwErrorIndex = 0;
		}
	else
		{
		// bump ring buffer index for NEXT error code
		++pgwErrorIndex;
		}


	// make sure the NEXT error code is always ZERO
    pgwRuntimeErrorBuffer[pgwErrorIndex] = 0;
}



// reset the RuntimeError buffer index
void ResetRuntimeError()
{

	pgwErrorIndex = 0;						// restart the error buffer
	pgwRuntimeErrorBuffer[0] = 0;

}
