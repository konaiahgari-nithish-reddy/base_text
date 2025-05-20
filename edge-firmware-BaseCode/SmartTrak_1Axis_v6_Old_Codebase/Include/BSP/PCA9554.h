// *************************************************************************************************
//										P C A 9 5 5 4 . h
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	PCA9554 I/O Expander Functions
//
// *************************************************************************************************

// These functions are NOT register level. They call into the PIC32 Peripheral library to
// handle most register access, especially initialization

#define	PCA9554_INPUT_PORT_REGISTER		0x00
#define	PCA9554_OUTPUT_PORT_REGISTER	0x01	// default/power up all 1
#define	PCA9554_POLARITY_REGISTER		0x02	// default/power up all 0
#define	PCA9554_CONFIGURATION_REGISTER	0x03	// default/power up all 1, inputs

#ifdef USE_PCA9554_IO							// enable in config.h ONLY if PCA9554 hardware is present
	BYTE GetInputSwitchState(void);
	void SetLEDState(BYTE cLEDState);
	BYTE GetLEDState(void);
#endif	// USE_PCA9554_IO


// end of PCA9554.h
