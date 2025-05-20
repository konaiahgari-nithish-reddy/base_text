// *************************************************************************************************
//										P C A 9 5 5 4 . c
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	PCA9554 I/O Expander Functions
//
// *************************************************************************************************

#include <GenericTypeDefs.h>

#include "config.h"				// compile time configuration definitions

#include "gsfstd.h"						// gsf standard #defines
#include "Debug.h"
#include "SmartTrak.h"

#include "HardwareProfile.h"

#include "LEDs.h"				// LED display handler function definition, used for stall recovery states

#include "I2CBus.h"
#include "DS3232.h"
#include "PCA9554.h"

#if defined (PLATFORM_PIC32_SK)
	#define	SWITCH_CONFIG_DATA				0x0F		// switches are inputs on lower nibl
	#define	LED_CONFIG_DATA					0x0F		// LEDs are are outputs on high nibl
#elif defined (PLATFORM_SMARTTRAK_V1)
	#define	SWITCH_CONFIG_DATA				0xFF		// switches are inputs
	#define	LED_CONFIG_DATA					0x00		// LEDs are outputs
#endif

#ifdef USE_PCA9554_IO							// enable in config.h ONLY if PCA9554 hardware is present

BYTE GetInputSwitchState(void)
{
	UINT8	cSwitchData = SWITCH_CONFIG_DATA;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	// set configuration to input (REQUIRED before EACH read, see TI PCA9554 data sheet, page 8)
	IGNORE_RETURN_VALUE write_i2c_device(SWITCH_IO_EXPANDER_I2C_ADDR, PCA9554_CONFIGURATION_REGISTER, &cSwitchData, 1);

	// invert the polarity of the Switch inputs, which are Active/Closed LOW
	IGNORE_RETURN_VALUE write_i2c_device(SWITCH_IO_EXPANDER_I2C_ADDR, PCA9554_POLARITY_REGISTER, &cSwitchData, 1);

	// read current switch values
	IGNORE_RETURN_VALUE read_i2c_device(SWITCH_IO_EXPANDER_I2C_ADDR, PCA9554_INPUT_PORT_REGISTER, &cSwitchData, 1);

	cSwitchData &= SWITCH_CONFIG_DATA;			// strip to bits initialized as inputs

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return cSwitchData;
}


void SetLEDState(BYTE cLEDState)
{
	UINT8	cConfigData  = LED_CONFIG_DATA;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	// set configuration to OUTPUT
	IGNORE_RETURN_VALUE write_i2c_device(LED_IO_EXPANDER_I2C_ADDR, PCA9554_CONFIGURATION_REGISTER, &cConfigData, 1);

	IGNORE_RETURN_VALUE write_i2c_device(LED_IO_EXPANDER_I2C_ADDR, PCA9554_OUTPUT_PORT_REGISTER, &cLEDState, 1);

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

}

BYTE GetLEDState()
{
	UINT8	cLEDData;
	BOOL	bRetVal;

	cLEDData = LED_CONFIG_DATA;

    // Enable the I2C bus
    I2CEnable(DS3232_I2C_BUS, TRUE);

	// set configuration to input (REQUIRED before EACH read, see TI PCA9554 data sheet, page 8)
	IGNORE_RETURN_VALUE write_i2c_device(LED_IO_EXPANDER_I2C_ADDR, PCA9554_CONFIGURATION_REGISTER, &cLEDData, 1);

	// read current switch values
	bRetVal = read_i2c_device(LED_IO_EXPANDER_I2C_ADDR, PCA9554_OUTPUT_PORT_REGISTER, &cLEDData, 1);
	// ==>> should clear the return value is read is not successful

    // Disable the I2C bus
    I2CEnable(DS3232_I2C_BUS, FALSE);

	return cLEDData;

}

#endif		//  USE_PCA9554_IO


// end of PCA9554.c