#ifndef _OpticalFlow_H
#define _OpticalFlow_H
#include "stm32f4xx.h"
#define OpticalFlow_I2C_Addr 0x84 //IIC 写入时的地址字节数据，+1 为读取                      

                     



void OpticalFlow_init(void);
void GetData_OpticalFlow(uint8_t REG_Address, uint8_t*  Rece_Data);

#endif  
