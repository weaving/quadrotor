#ifndef __I2C_H  
#define __I2C_H  
#include "sys.h"
  
  
void MS5611_SendCommand(u8 SlaveAddress,u8 REG_data);
void I2C_ByteWrite(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
void I2C_Configuration(void);
uint8_t I2C_ByteRead(uint8_t addr , uint8_t REG_Address);
uint8_t* I2C_GetData(uint8_t addr,uint8_t REG_data, uint8_t length, uint8_t*  Rece_Data);

void I2C1_Configuration(void);
uint8_t I2C1_ByteRead(uint8_t addr , uint8_t REG_Address);
uint8_t* I2C1_GetData(uint8_t addr,uint8_t REG_data, uint8_t length, uint8_t*  Rece_Data);

void MS5611_SendCommand(u8 SlaveAddress,u8 REG_data);
uint32_t MS5611_ReadADC(uint8_t addr , uint8_t REG_Address);
uint16_t MS5611_ReadProm(uint8_t addr , uint8_t REG_Address);
#endif 


