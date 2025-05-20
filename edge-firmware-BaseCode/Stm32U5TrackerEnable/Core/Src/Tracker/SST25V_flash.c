#include "SST25V_flash.h"

#define FLASH_READ_SLOW				0x03
#define FLASH_READ_FAST				0x0B
#define FLASH_ERASE_4K				0x20
#define FLASH_ERASE_32K				0x52
#define FLASH_ERASE_64K				0xD8
#define FLASH_ERASE_CHIP			0x60 //or 0xC7
#define FLASH_BYTE_PROGRAM			0x02
#define FLASH_AAI					0xAD
#define FLASH_READ_STATUS			0x05
#define FLASH_ENABLE_WRITE_STATUS	0x50
#define FLASH_WRITE_STATUS			0x01
#define FLASH_WRITE_ENABLE			0x06
#define FLASH_WRITE_DISABLE			0x04
#define FLASH_READ_ID				0x90 //or 0xAB
#define FLASH_READ_JEDEC_ID			0x9F
#define FLASH_ENABLE_SO				0x70
#define FLASH_DISABLE_SO			0x80

#define FLASH_SBIT_BUSY				0x01
#define FLASH_SBIT_WRITE_ENABLE		0x01

SST25_JEDEC_READ_ID SST25_jedec_read_id(void)
{
	uint8_t cmd = FLASH_READ_JEDEC_ID;
	SST25_JEDEC_READ_ID r_buffer;
	SST25_start();
	flash_SPI_write(&cmd, 1);
	flash_SPI_read(&r_buffer.Manufacturers_ID, 1);
	flash_SPI_read(&r_buffer.Memory_Type, 1);
	flash_SPI_read(&r_buffer.Memory_Capacity, 1);	
	SST25_end();
	return r_buffer;
}

SST25_RDID SST25_read_id(void)
{
	uint8_t cmd[] =
	{ 
		FLASH_READ_ID,
		0x0,
		0x0,
		0x1
	};
	SST25_RDID r_buffer;
	SST25_start();
	flash_SPI_write(&cmd, 4);
	flash_SPI_read(&r_buffer, 4);
	SST25_end();
	return r_buffer;
}

void SST25_init(void)
{
	SST25_end();
//	SST25_delay_ns(1);
}

void SST25_start(void)
{
	HAL_GPIO_WritePin(FLASH_SS_GPIO_Port, FLASH_SS_Pin, GPIO_PIN_RESET); 		//CE low
//	SST25_delay_ns(1);
}

void SST25_end(void)
{
	HAL_GPIO_WritePin(FLASH_SS_GPIO_Port, FLASH_SS_Pin, GPIO_PIN_SET);		//CE high
//	SST25_delay_ns(1);
}

void SST25_wait_while_busy(void)
{
	while (SST25_get_status() & FLASH_SBIT_BUSY) ;
}

void SST25_write_status(uint8_t status)
{
	SST25_enable_write_status();
	//SST25_start();
	//flash_SPI_read_write_byte(FLASH_ENABLE_WRITE_STATUS);
	//SST25_end();
	SST25_start();
	uint8_t cmd[] = { FLASH_WRITE_STATUS, status };
	flash_SPI_write(cmd, 2);
	SST25_end();
	//TODO: Disable write_statur_registere
}

void flash_SPI_write(uint8_t* pData, uint16_t size)
{
	HAL_SPI_Transmit(&SST25_hspi, pData, size, HAL_MAX_DELAY); 
}

uint8_t flash_SPI_read_write_byte(uint8_t data)
{
	uint8_t received;
	HAL_SPI_TransmitReceive(&SST25_hspi, &data, &received, 1, 1000);
	return received;
}

void SST25_StartCommandEnd_sequense(uint8_t command2send)
{
	SST25_start();
	flash_SPI_read_write_byte(command2send);
	SST25_end();
}

void SST25_write_enable(void)
{
	SST25_StartCommandEnd_sequense(FLASH_WRITE_ENABLE);
}

void SST25_write_disable(void)
{
	SST25_StartCommandEnd_sequense(FLASH_WRITE_DISABLE);
}

void SST25_enable_hardware_EOW(void)
{
	SST25_StartCommandEnd_sequense(FLASH_ENABLE_SO);
}

void SST25_disable_hardware_EOW(void)
{
	SST25_StartCommandEnd_sequense(FLASH_DISABLE_SO);
}

void SST25_chip_erase_sequence(void)
{
	SST25_StartCommandEnd_sequense(FLASH_ERASE_CHIP);	
}	

void SST25_enable_write_status(void)
{
	SST25_StartCommandEnd_sequense(FLASH_ENABLE_WRITE_STATUS);
}

void flash_SPI_read(uint8_t *read_buffer, uint16_t size)
{
	HAL_SPI_Receive(&SST25_hspi, read_buffer, size, 1000);
}

SST25_STATUS_REGISTER SST25_RDSR(void)
{
	uint8_t cmd = FLASH_READ_STATUS;
	SST25_STATUS_REGISTER r_buffer;
	SST25_start();
	flash_SPI_write(&cmd, 1);
	flash_SPI_read(&r_buffer, 1);
	SST25_end();
	return r_buffer;
}

uint8_t SST25_get_status(void)
{
	HAL_StatusTypeDef hal_result;
	SST25_start();
	uint8_t cmd[] = { FLASH_READ_STATUS, 0x00 };
	uint8_t received[2];
	if ((hal_result = HAL_SPI_TransmitReceive(&SST25_hspi, cmd, received, 2, 1000)) != HAL_OK)
		Error_Handler();
	SST25_end();
	return received[1];
}

void SST25_read(uint32_t address, uint8_t *read_buffer, uint16_t size)
{
	address &= 0xFFFFFF;

	SST25_start();
	uint8_t cmd[] = {
		FLASH_READ_FAST,
		(address >> 16) & 0xFF,
		(address >> 8) & 0xFF,
		address & 0xFF,
		0xFF // Dummy byte
	};
	flash_SPI_write(cmd, 5);
	flash_SPI_read(read_buffer, size);
	SST25_end();
}

void SST25_read_slow(uint32_t address, uint8_t *read_buffer, uint16_t size)
{
	address &= 0xFFFFFF;
	SST25_start();
	uint8_t cmd[] = {
		FLASH_READ_SLOW,
		(address >> 16) & 0xFF,
		(address >> 8) & 0xFF,
		address & 0xFF
	};
	flash_SPI_write(cmd, 4);
	flash_SPI_read(read_buffer, size); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	SST25_end();
}

void SST25_write_byte(uint32_t address, uint8_t data)
{
	SST25_write_status(0x00);    // Clear all sector protection
	SST25_write_enable();
	//	if (!(SST25_get_status() & FLASH_SBIT_WRITE_ENABLE))
	//	{
	//		SST25_write_enable();
	//	}
	address &= 0xFFFFFF;

	SST25_start();
	uint8_t cmd[] = {
		FLASH_BYTE_PROGRAM,
		(address >> 16) & 0xFF,
		(address >> 8) & 0xFF,
		address & 0xFF,
		data
	};
	flash_SPI_write(cmd, 5);
	SST25_end();
	SST25_wait_while_busy();
	SST25_write_disable();
}

void SST25_write(uint32_t address, uint8_t *data, uint16_t size)
{
	
	SST25_write_status(0x00);     // Clear all sector protection
	SST25_write_enable();
	//if (!(SST25_get_status() & FLASH_SBIT_WRITE_ENABLE))
	//	SST25_write_enable();
	SST25_enable_hardware_EOW();

	address &= 0xFFFFFF;
	uint8_t address_cmd[] = {
		FLASH_AAI,
		(address >> 16) & 0xFF,
		(address >> 8) & 0xFF,
		address & 0xFF,
	};
	uint8_t data_cmd[3] = { FLASH_AAI, 0x00, 0x00 };
	uint16_t written = 0;
	uint16_t remaining;

	while ((remaining = size - written) > 1)
	{
		SST25_start();
		if (!written)
			flash_SPI_write(address_cmd, 4);

		data_cmd[1] = data[written++];
		data_cmd[2] = data[written++];

		if (written <= 2)
			flash_SPI_write(data_cmd + 1, 2);
		else
			flash_SPI_write(data_cmd, 3);
		SST25_end();

		SST25_start();
		while (!HAL_GPIO_ReadPin(FLASH_MISO_GPIO_Port, FLASH_MISO_Pin)) ;
		SST25_end();
	}
	
	SST25_write_disable();
	//SST25_start();
	//flash_SPI_read_write_byte(FLASH_WRITE_DISABLE);
	//SST25_end();
	SST25_disable_hardware_EOW();
	//SST25_start();
	//flash_SPI_read_write_byte(FLASH_DISABLE_SO);
	//SST25_end();
	if (remaining)
		SST25_write_byte(address + written, data[written]);
}

void SST25_sector_erase_4K(uint32_t address)
{
	SST25_sector_erase(address, FLASH_ERASE_4K);
}

void SST25_sector_erase_32K(uint32_t address)
{
	SST25_sector_erase(address, FLASH_ERASE_32K);
}

void SST25_sector_erase_64K(uint32_t address)
{
	SST25_sector_erase(address, FLASH_ERASE_64K);
}

void SST25_sector_erase(uint32_t address, uint8_t amount)
{
	address &= 0xFFFFFF;
	SST25_write_status(0x00);  // Clear all sector protection
	SST25_write_enable();
	SST25_start();
	uint8_t cmd[] = {
		amount,
		(address >> 16) & 0xFF,
		(address >> 8) & 0xFF,
		address & 0xFF
	};
	flash_SPI_write(cmd, 4);
	SST25_end();
	SST25_wait_while_busy();
	//while (SST25_get_status() & FLASH_SBIT_BUSY);
	SST25_write_disable();
}

void SST25_sector_erase_ChipErase(void)
{
		SST25_write_status(0x00);     // Clear all sector protection
		SST25_write_enable();
		SST25_chip_erase_sequence();
		//SST25_start();
		//flash_SPI_read_write_byte(FLASH_ERASE_CHIP);
		//SST25_end();
		SST25_wait_while_busy();
		//while (SST25_get_status() & FLASH_SBIT_BUSY);
		SST25_write_disable();
}
