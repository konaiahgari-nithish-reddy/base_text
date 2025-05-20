void StopTransferAfterWrite( void )
{
    I2C_STATUS  status;

    #ifdef USE_I2C_STOP_TRIGGER
	Trigger1Level(1);				// trigger to allow viewing this event on a scope
    #endif

    I2CClearStatus(DS3232_I2C_BUS, I2C_STOP);

    // Send the Stop signal
    I2CStop(DS3232_I2C_BUS);			// note that this is once again OUTSIDE the loop!

    // Wait for the signal to complete
    do
    {
	status = I2CGetStatus(DS3232_I2C_BUS);

    } while ( !(status & I2C_STOP) );		// BLOCKING_WAIT

    #ifdef USE_I2C_STOP_TRIGGER
	Trigger1Level(0);				// trigger to allow viewing this event on a scope
    #endif
}





void StopTransferAfterRead( void )
{
    I2C_STATUS  status;

    #ifdef USE_I2C_STOP_TRIGGER
	Trigger1Level(1);				// trigger to allow viewing this event on a scope
	#endif

    I2CClearStatus(DS3232_I2C_BUS, I2C_STOP);

    // Send the Stop signal
    I2CStop(DS3232_I2C_BUS);

    // Wait for the signal to complete
    status = I2CGetStatus(DS3232_I2C_BUS);

    #ifdef USE_I2C_STOP_TRIGGER
	Trigger1Level(0);				// trigger to allow viewing this event on a scope
    #endif
}
