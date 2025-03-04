#include <stdio.h>
#include "ADS1256.h"
#include "driver/gpio.h"
#include "esp32c3/rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"



/**
 * @brief Initialize the GPIO pins for ADS1256.
 */
void ads1256_gpio_init(void)
{
	gpio_config_t io_conf; 

	io_conf.intr_type = GPIO_INTR_DISABLE;  
	io_conf.mode = GPIO_MODE_OUTPUT;       
	io_conf.pin_bit_mask = RESET_PIN_SEL;        
	io_conf.pull_down_en = 0;         
	io_conf.pull_up_en = 1;                     

	gpio_config(&io_conf); 

	io_conf.pin_bit_mask = DRDY_PIN_SEL;   
	io_conf.mode = GPIO_MODE_INPUT;             
	gpio_config(&io_conf);      

	RESET1;
}

/**
 * @brief Write a register value to ADS1256.
 * 
 * @param regaddr The register address.
 * @param databyte The data byte to write.
 */
void write_reg(unsigned char regaddr,unsigned char databyte)
{
	CS0;
	while(DRDY);
	spi_write_byte(CMD_WREG | (regaddr & 0x0F));
	spi_write_byte(0x00);
	spi_write_byte(databyte);
	CS1;
}

/**
 * @brief Initialize ADS1256.
 */
void ads1256_init(void)
{
	CS0;//片选拉低
	while(DRDY){

	}
	spi_write_byte(CMD_SELFCAL);//发送自校准命令
	CS1;//片选拉高

	write_reg(REG_STATUS,0x06);
	write_reg(REG_ADCON,0x00);
	write_reg(REG_DRATE,DRATE_1000SPS);
	write_reg(REG_IO,0x00);

	CS0;//片选拉低
	while(DRDY){

	}
	spi_write_byte(CMD_SELFCAL);//发送自校准命令
	CS1;//片选拉高

	// Check if ADS1256 is working properly
	// unsigned char id = read_reg(REG_ID);
	// printf("ID: %x\n", id);
	// if (id != ADS1256_ID)
	// {
	//     printf("ADS1256 initialization failed. ID mismatch.\n");
	//     // Handle the error or exit the program
	// }
}

/**
 * @brief Read a register value from ADS1256.
 * 
 * @param regaddr The register address.
 * @return The value read from the register.
 */
unsigned char read_reg(unsigned char regaddr)
{
	CS0;
	while(DRDY);
	spi_write_byte(CMD_RREG | (regaddr & 0x0F));
	spi_write_byte(0x00);
	spi_write_byte(0x01);
	ets_delay_us(1);
	unsigned char value = spi_recive_byte();
	CS1;
	return value;
}

/**
 * @brief Read data from ADS1256.
 * 
 * @return The data read from ADS1256.
 */
unsigned int ads1256_read_data(void)  
{
	uint32_t sum=0;
	CS0;
	while(DRDY);  
	ets_delay_us(1);             
	spi_write_byte(CMD_SYNC);
	spi_write_byte(CMD_WAKEUP); 
	while(DRDY)
	{}               
	spi_write_byte(CMD_RDATA);
	ets_delay_us(50);
	sum |= (spi_recive_byte()<<16);
	sum |= (spi_recive_byte()<< 8);
	sum |= spi_recive_byte();
	// sum &= 0xfffe00;
	if (sum>0x7FFFFF)           // if MSB=1, 
	{
		sum -= 0x1000000;       // do 2's complement
	}      
	CS1;
	return sum;
}

/**
 * @brief Initialize the software SPI pins.
 */
void soft_spi_init(void)
{
	gpio_config_t io_conf; 

	io_conf.intr_type = GPIO_INTR_DISABLE;  
	io_conf.mode = GPIO_MODE_OUTPUT;       
	io_conf.pin_bit_mask = CS_PIN_SEL | CLK_PIN_SEL | DIN_PIN_SEL;        
	io_conf.pull_down_en = 0;         
	io_conf.pull_up_en = 0;                     
	gpio_config(&io_conf);  

	io_conf.pin_bit_mask = DOUT_PIN_SEL;   
	io_conf.mode = GPIO_MODE_INPUT;            
	io_conf.pull_down_en = 0;         
	io_conf.pull_up_en = 0;                     
	gpio_config(&io_conf);
	CS1;
	CLK0;
	DOUT1; 
	DIN1;
}

/**
 * @brief Write a byte to the SPI bus.
 * 
 * @param TxData The byte to write.
 */
void spi_write_byte(uint8_t TxData)
{
	uint8_t i;
	ets_delay_us(1);
	for(i = 0; i < 8; i++)
	{
		if (TxData & 0x80)
		{
			DIN1;
		}
		else
		{
			DIN0;
		}
		CLK1;
		ets_delay_us(2);
		TxData <<= 1;
		CLK0;
		ets_delay_us(1);
	}
}

/**
 * @brief Receive a byte from the SPI bus.
 * 
 * @return The byte received.
 */
uint8_t spi_recive_byte(void)
{
	uint8_t i;
	uint8_t read = 0;
	ets_delay_us(1);
	for (i = 0; i < 8; i++)
	{
		CLK1;
		ets_delay_us(2);
		read = read<<1;
		CLK0;
		ets_delay_us(1);
		if (DOUT)
		{
			read++;
		}
		ets_delay_us(1);
	}
	return read;
}


