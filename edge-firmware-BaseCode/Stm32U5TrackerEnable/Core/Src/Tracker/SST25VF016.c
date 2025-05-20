#include "SST25VF016.h"
#include "SST25V_flash.h"
#include <stdio.h>
#include <string.h>
#include "stm32u5xx_hal.h"
#include "PeripheralCallbacks.h"
#include "main.h"

/************************************************************************
 * Function: SST25Init
 *
 * Overview: this function setup SPI and IOs connected to SST25
 *
 * Input: none
 *
 * Output: none
 *
 ************************************************************************/
void SST25Init(void)
{
	SST25_init();
}

/************************************************************************
 * Function: SST25ReadID()
 *
 * Overview: this function reads JEDEC ID, Manufacturer, Device Type
 *
 * Input: none
 *
 * Output: TRUE is as expected, otherwise FALSE

 * See SST25VF016 datasheet Page 20
 *
 ************************************************************************/
BOOL SST25ReadID(BYTE *pData)
{
	BOOL	bRetVal = TRUE;

	SST25_JEDEC_READ_ID rxData;
	rxData = SST25_jedec_read_id();

	// Parse the received IDs
	*pData = rxData.Manufacturers_ID;

	if (*pData != ID_JEDEC_SST)			// verify JEDEC Manufacturer ID
		bRetVal = FALSE;

	pData++;								// bump output buffer

	*pData = rxData.Memory_Type;

	if (*pData != ID_SPI_FLASH)			// verify device format
		bRetVal = FALSE;

	pData++;								// bump output buffer

	*pData = rxData.Memory_Capacity;

	if (*pData != ID_SST25VF016)		// verify specific device type
		bRetVal = FALSE;

	return bRetVal;
}

/************************************************************************
 * Function: void SST25WriteByte(BYTE data, DWORD address)
 *
 * Overview: this function writes a byte to the address specified
 *
 * Input: data to be written and address
 *
 * Output: none
 *
 ************************************************************************/
void SST25WriteByte(BYTE data, uint32_t address)
{
	SST25_write_byte(address, data);
}

/************************************************************************
 * Function: BYTE SST25ReadByte(DWORD address)
 *
 * Overview: this function reads a byte from the address specified
 *
 * Input: address
 *
 * Output: data read
 *
 ************************************************************************/
BYTE SST25ReadByte(uint32_t address)
{
	BYTE    temp;

	SST25_read(address, &temp, 1);

	return (temp);
}

/************************************************************************
 * Function: void SST25WriteWord(WODR data, DWORD address)
 *
 * Overview: this function writes a 16-bit word to the address specified
 *
 * Input: data to be written and address
 *
 * Output: none
 *
 ************************************************************************/
void SST25WriteWord(WORD data, uint32_t address)
{
	uint8_t txdata[2];

	txdata[0] = (data>>8) & 0xFF;
	txdata[1] = data & 0xFF;

	SST25_write(address, txdata, sizeof(txdata));
}

/************************************************************************
 * Function: WORD SST25ReadWord(DWORD address)
 *
 * Overview: this function reads a 16-bit word from the address specified
 *
 * Input: address
 *
 * Output: data read
 *
 ************************************************************************/
WORD SST25ReadWord(uint32_t address)
{
	unsigned short int temp;

	SST25_read(address, &temp, sizeof(temp));

	return temp;

}

/************************************************************************
 * Function: BYTE SST25WriteArray(DWORD address, BYTE* pData, nCount)
 *
 * Overview: this function writes a data array at the address specified
 *
 * Input: flash memory address, pointer to the data array, data number
 *
 * Output: return 1 if the operation was successfull
 *
 ************************************************************************/
BYTE SST25WriteArray(DWORD address, BYTE *pData, WORD nCount)
{
	SST25_write(address, pData, nCount);

	return (1);
}

/************************************************************************
 * Function: void SST25ReadArray(DWORD address, BYTE* pData, nCount)
 *
 * Overview: this function reads data into buffer specified
 *
 * Input: flash memory address, pointer to the data buffer, data number
 *
 ************************************************************************/
void SST25ReadArray(DWORD address, BYTE *pData, WORD nCount)
{
	SST25_read(address, pData, nCount);
}

/************************************************************************
 * Function: void SST25ChipErase(void)
 *
 * Overview: chip erase
 *
 * Input: none
 *
 ************************************************************************/
void SST25ChipErase(void)
{
	SST25_sector_erase_ChipErase();
}

/************************************************************************
 * Function: void SST25SectorErase(DWORD address)
 *
 * Overview: this function erases a 4Kb sector
 *
 * Input: address within sector to be erased
 *
 * Output: none
 *
 ************************************************************************/
void SST25SectorErase(uint32_t address)
{
	SST25_sector_erase_4K(address);
}


