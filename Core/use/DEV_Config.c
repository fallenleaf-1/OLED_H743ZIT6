/******************************************************************************
**************************Hardware interface layer*****************************
* | file      		:	DEV_Config.c
* |	version			:	V1.0
* | date			:	2020-06-17
* | function		:	Provide the hardware underlying interface	
******************************************************************************/
#include "DEV_Config.h"
#include "spi.h"
#include <stdio.h>		//printf()
#include <string.h>
#include <stdlib.h>


/********************************************************************************
function:	System Init
note:
	Initialize the communication method
********************************************************************************/
uint8_t System_Init(void)
{
	OLED_RST_0;
	HAL_Delay(100);
	OLED_RST_1;
	HAL_Delay(100);
  return 0;
}

void System_Exit(void)
{
	OLED_CS_1;
}
/********************************************************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) : 
		HAL library hardware SPI
		Register hardware SPI
		Gpio analog SPI
	I2C_Write_Byte(value, cmd):
		HAL library hardware I2C
********************************************************************************/
uint8_t SPI4W_Write_Byte(uint8_t value)
{
    HAL_SPI_Transmit(&hspi1, &value, 1, 500);
    return 0;
}

/*void I2C_Write_Byte(uint8_t value, uint8_t Cmd)
{
    int Err;
    uint8_t W_Buf[2] ;
    W_Buf[0] = Cmd;
    W_Buf[1] = value;
    if(HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADR << 1) | 0X00, W_Buf, 2, 0x10) != HAL_OK) {
        Err++;
        if(Err == 1000) {
            printf("send error\r\n");
            return ;
        }
    }
}*/

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
    HAL_Delay(xms);
}

void Driver_Delay_us(uint32_t xus)
{
    int j;
    for(j=xus*120; j > 0; j--)
	{
		__NOP();
	}
	
}
