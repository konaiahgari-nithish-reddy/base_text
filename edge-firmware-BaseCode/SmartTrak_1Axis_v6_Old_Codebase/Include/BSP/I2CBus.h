// *************************************************************************************************
//										I 2 C B u s . C
// *************************************************************************************************
//
//		Project:	SmartTrak Solar Panel Controller
//
//		Contains:	I2C Bus Function definitions
//
// *************************************************************************************************

BOOL InitializeI2CBus(void);
BOOL StartTransfer( BOOL restart );
BOOL TransmitOneByte( UINT8 data );
void StopTransfer( void );
void I2CReset(void);

// These functions are NOT register level. They call into the PIC32 Peripheral library to
// handle most register access, especially initialization

BOOL read_i2c_device(UINT8 cDeviceI2CAddress, UINT8 cSubAddress, UINT8 *ptrData, UINT8 cDataLen);
BOOL write_i2c_device(UINT8 cDeviceI2CAddress, UINT8 cSubAddress, UINT8 *ptrData, UINT8 cDataLen);

// end of I2CBus.h
