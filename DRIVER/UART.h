#include "stm32f4xx.h"
#include <stdio.h>

#ifndef _UART_H
#define _UART_H

/**
  * @brief  这个函数用来配置 串口1 
            使用PB6、PB7
			
			使用方法：
			调用UART_Config(9600);
			就可以配置9600波特率的串口1
  * @param  波特率
  * @retval None
  */

void UART_Config(int BaudRate) ;


void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
//int fputc(int ch, FILE *f);


#endif
