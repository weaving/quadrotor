#include "i2c.h"
#include "delay.h"  
#include "stm32f4xx_i2c.h"  
#include "sys.h"
#include "includes.h"
#include "flash_eeprom.h"
#include "OpticalFlow.h"


void OpticalFlow_init(void)
{
	I2C1_Configuration();
}

void GetData_OpticalFlow(uint8_t REG_Address, uint8_t*  Rece_Data)
{
		I2C1_GetData(OpticalFlow_I2C_Addr, REG_Address, 22, Rece_Data);
}
