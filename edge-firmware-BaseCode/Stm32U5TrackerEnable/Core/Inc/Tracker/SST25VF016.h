#include"GenericTypeDefs.h"
#include"gsfstd.h"
#include"stdint.h"

#ifndef __SST25VF016_H__
#define __SST25VF016_H__


/************************************************************************
* SST25 Commands
************************************************************************/
	#define SST25_CMD_WRITE				(unsigned)0x02		// write at 25MHz
	#define SST25_CMD_READ				(unsigned)0x03		// read at 25MHz
    #define SST25_CMD_HS_READ			(unsigned)0x0B		// read at 80MHz
    #define SST25_CMD_4K_ERASE			(unsigned)0x20
    #define SST25_CMD_32K_ERASE			(unsigned)0x52
    #define SST25_CMD_64K_ERASE			(unsigned)0xD8
    #define SST25_CMD_CHIP_ERASE		(unsigned)0x60		// erase entire chip
    #define SST25_CMD_BYTE_WRITE		(unsigned)0x02		// write one byte
    #define SST25_CMD_AUTO_ADDR_INC		(unsigned)0xAD		// auto address increment
    #define SST25_CMD_RD_STATUS_REG		(unsigned)0x05
    #define SST25_CMD_EN_WR_STATUS_REG	(unsigned)0x50		// enable status register write
    #define SST25_CMD_WR_STATUS_REG		(unsigned)0x01
	#define SST25_CMD_WR_EN				(unsigned)0x06		// write enable
	#define SST25_CMD_WR_DI				(unsigned)0x04		// write disable
	#define SST25_CMD_RD_ID				(unsigned)0x90		// read ID (alternate 0xAB)
	#define SST25_CMD_RD_JEDEC_ID		(unsigned)0x9F		// read Jedec ID
	#define SST25_CMD_EBSY				(unsigned)0x70		// Enable SO to output RY/BY# during AAI programming
	#define SST25_CMD_DBSY				(unsigned)0x70		// Disable SO output RY/BY#

/************************************************************************
* SST25 IDs
************************************************************************/
#define	ID_JEDEC_SST		0xBF
#define	ID_SPI_FLASH		0x25
#define	ID_SST25VF016		0x41

/************************************************************************
* Macro: SST25CSLow()
*
* Preconditions: CS IO must be configured as output
*
* Overview: this macro pulls down CS line
*
* Input: none
*
* Output: none
*

************************************************************************/
/*i comment
#define SST25CSLow()    SST25_CS_LAT = 0;
*/
#define SST25CSLow() HAL_GPIO_WritePin(Flash_SPI2_CS_GPIO_Port, Flash_SPI2_CS_Pin, GPIO_PIN_RESET);
/************************************************************************
* Macro: SST25CSHigh()
*
* Preconditions: CS IO must be configured as output
*
* Overview: this macro set CS line to high level
*
* Input: none
*
* Output: none
*
************************************************************************/
/*i comment  
  #define SST25CSHigh()   SST25_CS_LAT = 1;
*/
#define SST25CSHigh() HAL_GPIO_WritePin(Flash_SPI2_CS_GPIO_Port, Flash_SPI2_CS_Pin, GPIO_PIN_SET);
/************************************************************************
* Function: SST25Init()
*
* Overview: this function setups SPI and IOs connected to SST25
*
* Input: none
*
* Output: none
*
************************************************************************/
void    SST25Init(void);

/************************************************************************
* Function: SST25ReadID()
*
* Overview: this function reads JEDEC ID, Manufacturer, Device Type
*
* Input: none
*
* Output: TRUE is as expected, otherwise FALSE
*
************************************************************************/
BOOL SST25ReadID(BYTE *pData);

/************************************************************************
* Function: BYTE SST25IsWriteBusy(void)
*
* Overview: this function reads status register and chek BUSY bit for write operation
*
* Input: none
*
* Output: non zero if busy
*
************************************************************************/
BYTE    SST25IsWriteBusy(void);

/************************************************************************
* Function: void SST25WriteByte(BYTE data, DWORD address)
*
* Overview: this function writes a byte to the address specified
*
* Input: byte to be written and address
*
* Output: none
*
************************************************************************/
void   SST25WriteByte(BYTE data,DWORD address);

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
BYTE    SST25ReadByte(DWORD address);

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
void    SST25WriteWord(WORD data, DWORD address);

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
WORD    SST25ReadWord(DWORD address);

/************************************************************************
* Function: SST25WriteEnable()
*
* Overview: this function allows writing into SST25. Must be called
*           before every write/erase command
*
* Input: none
*
* Output: none
*
************************************************************************/
void    SST25WriteEnable(void);

/************************************************************************
* Function: BYTE SST25WriteArray(DWORD address, BYTE* pData, nCount)
*
* Overview: this function writes data array at the address specified
*
* Input: flash memory address, pointer to the data array, data number
*
* Output: return 1 if the operation was successfull
*
************************************************************************/
BYTE    SST25WriteArray(DWORD address, BYTE *pData, WORD nCount);

/************************************************************************
* Function: void SST25ReadArray(DWORD address, BYTE* pData, nCount)
*
* Overview: this function reads  data into buffer specified
*
* Input: flash memory address, pointer to the buffer, data number
*
************************************************************************/
void    SST25ReadArray(DWORD address, BYTE *pData, WORD nCount);

/************************************************************************
* Function: void SST25ChipErase(void)
*
* Overview: chip erase
*
* Input: none
*
************************************************************************/
void    SST25ChipErase(void);

/************************************************************************
* Function: void SST25ResetWriteProtection()
*
* Overview: this function reset write protection bits
*
* Input: none
*
* Output: none
*
************************************************************************/
void    SST25ResetWriteProtection(void);

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
void    SST25SectorErase(DWORD address);
#endif //_SST25VF016_H

