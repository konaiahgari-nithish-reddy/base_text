// *************************************************************************************************
//										I 2 C B u s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	I2C Bus Functions
//
// *************************************************************************************************

#ifndef __32MX360F512L__
	#error	Incorrect Processor Type Selected; code is for PIC32MX360F512L
#endif

//-----------------------------------------------------------------------------
//								#include files
//-----------------------------------------------------------------------------
#include <GenericTypeDefs.h>
#include "config.h"						// compile time configuration definitions

//lint -e765							error 765: (Info -- external function could be made static)
//lint -e14								error 14: (Error -- Symbol 'foo' previously defined (line moo, file yoo.c, module goo.c))
#include <plib.h>						// Microchip PIC32 peripheral library main header
//lint +e14


#include "gsfstd.h"						// gsf standard #defines
#include "HardwareProfile.h"

#include "Debug.h"
#include "SmartTrak.h"					// Project wide definitions

#include "HardwareProfile.h"
#include "TimeDelay.h"
#include "SerialTimer.h"

#include "EventFlags.h"					// event flags from all sources

#include "I2C.h"
#include "I2CBus.h"



#ifdef PIC32_STARTER_KIT
    #include <stdio.h>
#endif


enum tagI2CErrors
{
	I2C_ERROR_NONE = I2C_ERROR_BASE,
	I2C_ERROR_UNEXPECTED_TICK,					// 1 unexpected timer tick event
	I2C_ERROR_UNEXPECTED_EVENT,					// 2 unexpected event
	I2C_ERROR_INVALID_STATE,					// 3 not a valid state
	I2C_ERROR_INVALID_SUBSTATE,					// 4 not a valid state
	I2C_ERROR_INVALID_CLOCK_FREQ,				// 5 clock setting is not valid
	I2C_ERROR_BUS_IDLE_TIMEOUT,					// 6 I2C Bus did not go idle
	I2C_ERROR_START_TIMEOUT,					// 7 Start condition timeout
	I2C_ERROR_TX_READY_TIMEOUT,					// 8 TX ready for data timeout
	I2C_ERROR_TX_TIMEOUT,						// 9 TX timeout
	I2C_ERROR_MASTER_BUS_COLLISION,				// A collision during bus arbitration
	I2C_ERROR_STOP_TIMEOUT,						// B Start condition timeout
	I2C_ERROR_NO_ACK,							// C byte not acknowledged
	I2C_ERROR_RX_TIMEOUT,						// D RX timeout
	I2C_ERROR_BUS_RESET,						// E I2CReset called

	I2C_FSM_UNPROCESSED_EVENT = I2C_ERROR_BASE + 0x0F
};


// ****************************************************************************
// ****************************************************************************
// Local Support Routines
// ****************************************************************************
// ****************************************************************************

BOOL InitializeI2CBus(void)
{

    UINT32              actualClock;

    // Initialize debug messages (when supported)
    //DBINIT();

	// configure I2C
	#if defined(USE_I2C_NORMAL_CLOCK)
		I2CConfigure(DS3232_I2C_BUS, I2C_ENABLE_SLAVE_CLOCK_STRETCHING);
	#elif defined (USE_I2C_FAST_CLOCK)
		I2CConfigure(DS3232_I2C_BUS, I2C_ENABLE_SLAVE_CLOCK_STRETCHING | I2C_ENABLE_HIGH_SPEED);
	#endif
	
    // Set the I2C baudrate
    actualClock = I2CSetFrequency(DS3232_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);

    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
		RuntimeError(I2C_ERROR_INVALID_CLOCK_FREQ);
		//DBPRINTF("Error: I2C1 clock frequency (%u) error exceeds 10%%.\n", (unsigned)actualClock);
		return FALSE;
    }

	// do not enable the I2C bus here. It is enabled and disabled as needed by functions below.

	return TRUE;
}


/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the EEPROM.

  Description:
    This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition
    
  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling
    
  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/


//-------------------------------------------------------------------------------------------------------
// Function Declarations
//-------------------------------------------------------------------------------------------------------

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
		// repeat start is used within an I2C packet transfer
        I2CRepeatStart(DS3232_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
		StartI2CTimeout();							// start timeout
        while( !I2CBusIsIdle(DS3232_I2C_BUS) )		// BLOCKING_WAIT
		{
			if (GetI2CTimeoutState() IS_TRUE)		// check for timeout
			{
				StopI2CTimeout();					// timeout, end timeout counting
				RuntimeError(I2C_ERROR_BUS_IDLE_TIMEOUT);

				// exit within operation may leave the I2C bus in an unknown or awkward state
				I2CReset();							// clear the I2C bus by clearing any incomplete transfer from slave
				return FALSE;
			}
		}
		StopI2CTimeout();							// end timeout counting


        if(I2CStart(DS3232_I2C_BUS) != I2C_SUCCESS)
        {
			RuntimeError(I2C_ERROR_MASTER_BUS_COLLISION);
			//DBPRINTF("Error: Bus collision during transfer Start\n");

			// exit within operation may leave the I2C bus in an unknown or awkward state
			I2CReset();								// clear the I2C bus by clearing any incomplete transfer from slave
			return FALSE;
        }
    }

    // Wait for the signal to complete
	StartI2CTimeout();								// start timeout
    do
    {
        status = I2CGetStatus(DS3232_I2C_BUS);		// read bus status
		if (GetI2CTimeoutState() IS_TRUE)			// check for timeout
		{
			StopI2CTimeout();						// timeout, end timeout counting
			RuntimeError(I2C_ERROR_START_TIMEOUT);

			// exit within operation may leave the I2C bus in an unknown or awkward state
			I2CReset();								// clear the I2C bus by clearing any incomplete transfer from slave
			return FALSE;
		}

    } while ( !(status & I2C_START) );				// BLOCKING_WAIT_FOR_HARDWARE, wait for Start condition

	StopI2CTimeout();								// end timeout counting

    return TRUE;
}


/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
	StartI2CTimeout();								// start timeout
    while(!I2CTransmitterIsReady(DS3232_I2C_BUS))	// BLOCKING_WAIT_FOR_HARDWARE
	{
		if (GetI2CTimeoutState() IS_TRUE)			// check for timeout
		{
			StopI2CTimeout();						// timeout, end timeout counting
			RuntimeError(I2C_ERROR_TX_READY_TIMEOUT);

			// exit within operation may leave the I2C bus in an unknown or awkward state
			I2CReset();								// clear the I2C bus by clearing any incomplete transfer from slave
			return FALSE;
		}
	}
	StopI2CTimeout();								// end timeout counting

    // Transmit the byte
    if(I2CSendByte(DS3232_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
		RuntimeError(I2C_ERROR_MASTER_BUS_COLLISION);
		// DBPRINTF("Error: I2C Master Bus Collision\n");

		// exit within operation may leave the I2C bus in an unknown or awkward state
		I2CReset();									// clear the I2C bus by clearing any incomplete transfer from slave
        return FALSE;
    }

    // Wait for the transmission to finish
	StartI2CTimeout();								// start timeout
    while(!I2CTransmissionHasCompleted(DS3232_I2C_BUS))	// BLOCKING_WAIT_FOR_HARDWARE
	{
		if (GetI2CTimeoutState() IS_TRUE)			// check for timeout
		{
			StopI2CTimeout();						// timeout, end timeout counting
			RuntimeError(I2C_ERROR_TX_TIMEOUT);

			// exit within operation may leave the I2C bus in an unknown or awkward state
			I2CReset();								// clear the I2C bus by clearing any incomplete transfer from slave
			return FALSE;
		}
	}
	StopI2CTimeout();								// end timeout counting

    return TRUE;
}

/*
if (I2CReceivedDataIsAvailable(I2C1))
{
    I2CAcknowledgeByte(I2C1, TRUE);
    data = I2CGetByte(I2C1);
}
*/

/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the EEPROM.

  Description:
    This routine Stops a transfer to/from the EEPROM, waiting (in a 
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.
    
  Returns:
    None.
    
  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/


BOOL StopTransferAfterWrite( void )
{
    I2C_STATUS  status;

    #ifdef USE_I2C_STOP_AFTER_WRITE_TRIGGER
		Trigger1Level(1);							// trigger to allow viewing this event on a scope
    #endif

	// this implementation is similar to the original Microchip implementation
	// there is a single I2CStop(), followed by a loop waiting for the stop to occur
	I2CStop(DS3232_I2C_BUS);						// set the STOP condition

	StartI2CTimeout();								// start timeout
    do												// wait for the system to indicate the stop has occured (?)
	{
		// NOTE: a call to I2CClearStatus(DS3232_I2C_BUS, I2C_STOP) here will actually hang the system...
		status = I2CGetStatus(DS3232_I2C_BUS);		// read bus status
		if (GetI2CTimeoutState() IS_TRUE)			// check for timeout
		{
			StopI2CTimeout();						// timeout, end timeout counting
			RuntimeError(I2C_ERROR_START_TIMEOUT);

			// exit within operation may leave the I2C bus in an unknown or awkward state
			I2CReset();								// clear the I2C bus by clearing any incomplete transfer from slave
			return FALSE;
		}


	} while ( !(status & I2C_STOP) );				// BLOCKING_WAIT, Wait for the signal to complete

	StopI2CTimeout();								// end timeout counting

    #ifdef USE_I2C_STOP_AFTER_WRITE_TRIGGER
		Trigger1Level(0);							// trigger to allow viewing this event on a scope
    #endif

	return TRUE;
}


// This version of the stop function is quite different - and is only required for a STOP-after-READ.
// The below function does NOT work correctly for STOP-after-READ. The STOP condition is generated repeatedly
// without being recognized (I checked the signal lines) so the wait-for-STOP essentially kills the system.
// This implementation disables the I2C bus to force the lines into the STOP state.
void StopTransferAfterRead( void )
{
    I2C_STATUS  status;

    #ifdef USE_I2C_STOP_AFTER_READ_TRIGGER
		Trigger1Level(1);							// trigger to allow viewing this event on a scope
    #endif

	I2CStop(DS3232_I2C_BUS);						// set the STOP condition

	status = I2CGetStatus(DS3232_I2C_BUS);
	//  note that we do not DO anything with the status

    // DISABLE the I2C bus, forcing the bus lines into the IDLE (STOP) state  (SCL HI, SDA low-to-HI transition)
    I2CEnable(DS3232_I2C_BUS, FALSE);

	Delay10us(1);									// delay, no idea if this is adequate

    I2CEnable(DS3232_I2C_BUS, TRUE);			    // Enable the I2C bus

    #ifdef USE_I2C_STOP_AFTER_READ_TRIGGER
		Trigger1Level(0);							// trigger to allow viewing this event on a scope
    #endif

}


#ifdef DOES_NOT_WORK_CORRECTLY
// NOTE: this does not appear to work correctly; I2CStop is not recognized
void StopTransferAfterRead( void )
{
    I2C_STATUS  status;

    #ifdef USE_I2C_STOP_AFTER_READ_TRIGGER
		Trigger1Level(1);				// trigger to allow viewing this event on a scope
	#endif

    I2CClearStatus(DS3232_I2C_BUS, I2C_STOP);

    // Send the Stop signal
    I2CStop(DS3232_I2C_BUS);

    // Wait for the signal to complete
    status = I2CGetStatus(DS3232_I2C_BUS);
	
    #ifdef USE_I2C_STOP_AFTER_READ_TRIGGER
		Trigger1Level(0);				// trigger to allow viewing this event on a scope
    #endif
}
#endif

// This is structured as a generic I2C device write. The DS3232 register is simply the first data byte AFTER the device address
BOOL write_i2c_device(UINT8 cDeviceI2CAddress, UINT8 cSubAddress, UINT8 *ptrData, UINT8 cDataLen)
{
    UINT8               i2cData[256];				// much larger than needed for any current devices...
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 DataSz;
    //BOOL                Acknowledged;
    BOOL                Success = TRUE;
	int					i;

    //
    // Send the data to I2C Device
    //

    // Initialize the data buffer
	// create a properly formatted Slave Device Address + R/W byte.
	// I2C_FORMAT_7_BIT_ADDRESS() shifts the SlaveAddress LEFT one bit, so there is room for the R/W bit in the LSB
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, cDeviceI2CAddress, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;

	// format the rest of the data
    i2cData[1] = cSubAddress;		// secondary address, generally the address of register to be written

	// place data in the buffer AFTER the Device I2C address and subAddress
	for (i = 2; i < (cDataLen + 2); i++)
	{
	   i2cData[i] = *ptrData;		// data to be written
	   ++ptrData;					// bump data pointer
	}

	// set data length, INCLUDING the device address
    DataSz = cDataLen + 2;

    // Start the transfer to write data
    if( !StartTransfer(FALSE) )				// error cause reported internally, I2CReset() called internally
    {
        return FALSE;						// error exit
    }

    // Transmit all data
    Index = 0;
    while( Success && (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;

            // Verify that the byte was acknowledged, EACH byte must be acknowledged
			// any need to test more than once
            if(!I2CByteWasAcknowledged(DS3232_I2C_BUS))
            {
				RuntimeError(I2C_ERROR_NO_ACK);
				// DBPRINTF("Error: Sent byte was not acknowledged\n");
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }
    }

    // End the transfer
	// if data transfer was not successful, we still need to put the bus into a known state
    if (StopTransferAfterWrite() IS_FALSE)			// errors are reported internally, I2CReset() is called internally
	{
		return FALSE;
	}

    if(!Success)
    {
		I2CReset();									// clear the I2C bus by clearing any incomplete transfer from slave
        return FALSE;								// error exit
    }

#ifdef NOT_USED		// according to the PCA9554 specs, this is not necessary.

    // Wait for I2C Device to complete write process, by polling the ack status.
	// this could be a LONG time for an EEPROM; likely to be short for other devices
	// This is actually ANOTHER write attempt, but there is no data sent, just the I2C Address + R/W bit
    Acknowledged = FALSE;
    do
    {
        // Start the transfer to address the EEPROM
        if( !StartTransfer(FALSE) )
        {
	        return FALSE;							// error exit
        }

        // Transmit just the EEPROM's address
        if (TransmitOneByte(SlaveAddress.byte))		// transmit I2C Address + R/W bit, NO additional data
        {
            // Check to see if the byte was acknowledged
            Acknowledged = I2CByteWasAcknowledged(DS3232_I2C_BUS);
        }
        else
        {
            Success = FALSE;
        }

        // End the transfer (stop here if an error occured)
        StopTransferAfterWrite();

        if(!Success)
        {
	        return FALSE;							// error exit
        }

    } while (Acknowledged != TRUE);

#endif


	return TRUE;
}

BOOL read_i2c_device(UINT8 cDeviceI2CAddress, UINT8 cSubAddress, UINT8 *ptrData, UINT8 cDataLen)
{

    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 DataSz;
//    BOOL                Acknowledged;
    BOOL                Success = TRUE;
	UINT8				cDataToRead = cDataLen;		// number of data bytes to read

    //
    // Read the data back from the I2C Device
    //

    // Initialize the data buffer
	// create a properly formatted Slave Device Address + R/W byte.
	// I2C_FORMAT_7_BIT_ADDRESS() shifts the SlaveAddress LEFT one bit, so there is room for the R/W bit in the LSB
    I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, cDeviceI2CAddress, I2C_WRITE);
    i2cData[0] = SlaveAddress.byte;

	// format the rest of the data to be written
    i2cData[1] = cSubAddress;						// SRAM address to read
    DataSz = 2;

    // Start the transfer
    if( !StartTransfer(FALSE) )						// errors reported internally, I2CReset() is called internally
    {
        return FALSE;								// error exit
    }

    // Transmit I2C device address, Sub address (2 bytes)
    Index = 0;										// init data index
    while( Success & (Index < DataSz) )
    {
        // Transmit a byte
        if (TransmitOneByte(i2cData[Index]))
        {
            // Advance to the next byte
            Index++;
        }
        else
        {
            Success = FALSE;						// errors reported internally, I2CReset() is called internally
        }

        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(DS3232_I2C_BUS))
        {
			RuntimeError(I2C_ERROR_NO_ACK);
			// DBPRINTF("Error: Sent byte was not acknowledged\n");
            Success = FALSE;
        }
    }

    // Restart and send the device I2C address to switch to a read transfer
    if(Success)
    {
        // Send a Repeated Start condition
        if( !StartTransfer(TRUE) )
        {
			return FALSE;							// error exit, 	errors reported internally, I2CReset() is called internally
        }

        // Transmit the I2C Device address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, cDeviceI2CAddress, I2C_READ);
        if (TransmitOneByte(SlaveAddress.byte))		// if an error occurs, errors reported and I2CReset() called internally
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(DS3232_I2C_BUS))		// returns TRUE if byte is acknowledged
            {
				RuntimeError(I2C_ERROR_NO_ACK);
				// DBPRINTF("Error: Sent byte was not acknowledged\n");

                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;						// errors reported internally, I2CReset() is called internally
        }
    }



	if(I2CReceiverEnable(DS3232_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
	{
////		DBPRINTF("Error: I2C Receive Overflow\n");
		Success = FALSE;
	}

    // Read the data from the desired address
    while( Success && (cDataToRead != 0) )
    {

		StartI2CTimeout();							// start timeout
		// identify if the most recent byte received was an address byte or a data byte
		while(!I2CReceivedDataIsAvailable(DS3232_I2C_BUS))		// wait for available data, returns FALSE if received byte is data
		{
			if (GetI2CTimeoutState() IS_TRUE)		// check for timeout
			{
				StopI2CTimeout();					// timeout, end timeout counting
				RuntimeError(I2C_ERROR_RX_TIMEOUT);
				
				// exit within operation may leave the I2C bus in an unknown or awkward state
				I2CReset();							// clear the I2C bus by clearing any incomplete transfer from slave
				return FALSE;						// is this an acceptable exit path?
			}
		}
		StopI2CTimeout();							// end timeout counting

		--cDataToRead;								// decrement count of data left to read

		if (cDataToRead > 0)
		{
			I2CAcknowledgeByte(DS3232_I2C_BUS, TRUE);	// acknowlege received data byte, positive ACK indicates more data to be read
		}
		else
		{
			// send a NACK, to indicate this will be the last byte read
			I2CAcknowledgeByte(DS3232_I2C_BUS, FALSE);	// NOT acknowlege received data byte to end transfer
		}

		// read one byte of data; at this point the data is read from the MCU I2C Recieve Register
		*ptrData = I2CGetByte(DS3232_I2C_BUS);					// read back ONE byte

		++ptrData;									// bump returned data pointer for next byte (if any)
    }


	StopTransferAfterRead();

    if(!Success)
    {
		I2CReset();									// clear the I2C bus by clearing any incomplete transfer from slave
        return FALSE;								// error exit
    }

	return TRUE;

}


/*********************************************************************
* Function:     I2CReset()
*
* Input:		none
*
* Output:		true if success.
*
* Overview:		forced reset of EEPROM
*
* Note:			None
********************************************************************/

// NOTE: normally I2C lines are handled by actively pulling low, and releasing to be pulled high by an external pullup resistor.
// the SCL line must be driven high and low

#define	I2C_BIT_WIDTH_CNT	100		// width of I2C bit (10uS) in instruction cycles, for delay counter
#define I2C_RESET_BIT_CNT	9		// count of bits (clocks) generated during the Reset sequence

// FSM status values
#define	I2C_STATUS_OK		1
#define I2C_STATUS_BUSY		0
#define	I2C_STATUS_ERROR	-1
#define	I2C_STATUS_TIMEOUT	-2

unsigned char fI2CStatus = I2C_STATUS_OK;	// EEPROM I2C Status

void I2CReset()
{

	unsigned int uiDelayCtr = 0;
	unsigned char ucBitCtr;

	RuntimeError(I2C_ERROR_BUS_RESET);

    // I2C2CON : I2Cx CONTROL REGISTER
    // ----------------------------------------------------------
    // DISABLE the I2C bus, releasing the lines for discrete control
	// Does this?? forcing the bus lines into the IDLE (STOP) state  (SCL HI, SDA low-to-HI transition)
    I2CEnable(DS3232_I2C_BUS, FALSE);


	// re-establish the initial state of the I2C GPIO pins
	// set out levels LOW, for use when line is driven
	LATA &= (~I2C_GPIO_SDA_MASK);					// 0 = LOW, SDA LOW
	LATA |= I2C_GPIO_SCL_MASK;						// 1 = HIGH, SCL HI
	// set SDA open drain ON
	ODCA |= I2C_GPIO_SDA_MASK;						// 1 = open drain
	// set I2C SDA pin to INPUT, I2C SCL pin to OUTPUT
	TRISA |= I2C_GPIO_SDA_MASK;						// 1 = input
	TRISA &= (~I2C_GPIO_SCL_MASK);					// 0 = output

	fI2CStatus = I2C_STATUS_BUSY;					// set global status flag to indicate operation has not completed

    // ----------------------------------------------
    //				9 Clock Cycles
    // ----------------------------------------------
	// the sequence to be generated is:
	// 9 1-0-1 clocks on SCL, SDA left floating (pulled up by resistor)
	// 1 STOP, SDA pulled to 0, then floated to 1, while SCL remains 1

	for (ucBitCtr = 0; ucBitCtr < I2C_RESET_BIT_CNT; ucBitCtr++)
	{
		// set SCL LOW
		LATA &= (~I2C_GPIO_SCL_MASK);				// 0 = LOW, SCL LOW

		// delay for bit width, 10uS
		for (uiDelayCtr = 0; uiDelayCtr < I2C_BIT_WIDTH_CNT; uiDelayCtr++)
			;

		// set SCL HIGH
		LATA |= I2C_GPIO_SCL_MASK;					// 1 = HI, SCL HI

		// delay for bit width, 10uS
		for (uiDelayCtr = 0; uiDelayCtr < I2C_BIT_WIDTH_CNT; uiDelayCtr++)
			;
	}

    // ----------------------------------------------
    //				STOP
    // ----------------------------------------------
	// pull SDA (RB9) LOW to setup for STOP, while leaving SCL (RB8) HIGH
	// set I2C SDA pin to INPUT, I2C SCL pin to OUTPUT
	TRISA &= (~I2C_GPIO_SDA_MASK);					// 0 = output, already set to LOW in LATB

	for (uiDelayCtr = 0; uiDelayCtr < I2C_BIT_WIDTH_CNT; uiDelayCtr++)
		;

	TRISA |= I2C_GPIO_SDA_MASK;						// 1 = input, pulled HIGH by external pullup resistor

	for (uiDelayCtr = 0; uiDelayCtr < I2C_BIT_WIDTH_CNT; uiDelayCtr++)
		;

    // ----------------------------------------------
    //			Restore GPIO Lines
    // ----------------------------------------------
	// set I2C SCL, SDA pins to INPUT
	TRISA |= I2C_GPIO_MASK;						// 1 = input


	// successful completion
	fI2CStatus = I2C_STATUS_OK;					// set global status flag

}


// end of I2CBus.c
