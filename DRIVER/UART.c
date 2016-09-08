#include "stm32f4xx.h"
#include "UART.h"
#include <stdio.h>  

//static u8 RxBuffer[50];
//static u8 RxState = 0;
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
	

void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	uint8_t i;
	for(i=0;i<data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}


void UART_Config(int BaudRate)  
{  
	USART_InitTypeDef USART_InitStructure;  
	NVIC_InitTypeDef NVIC_InitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;  
	
  /* Enable the USARTx Interrupt */  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  NVIC_Init(&NVIC_InitStructure);  
	
  /* Enable GPIO clock */  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
  /* Enable UART clock */  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
   
  /* Connect PXx to USARTx_Tx*/  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);  
  /* Connect PXx to USARTx_Rx*/  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);  
  
  /* Configure USART Tx as alternate function  */  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
  /* Configure USART Rx as alternate function  */  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  
	USART_InitStructure.USART_BaudRate = BaudRate;  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_Parity = USART_Parity_No ;  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	USART_Init(USART1, &USART_InitStructure); /* Configure USART1 basic and asynchronous paramters */  
	USART_Cmd(USART1, ENABLE);   /* Enable USART1 */  

	
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  
}  


void USART1_IRQHandler(void)
{
//	char USART_Send_Buf[]="test";
//  int tmp;
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
//  {
       
//  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
//            tmp=USART_ReceiveData(USART1);
//            USART_SendData(USART1,tmp);
//  }

//  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
//  {   
//    /* Write one byte to the transmit data register */
//    //USART_SendData(USART1, ok);


//  }
		
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}

	//接收中断 (接收寄存器非空) 
//	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
//	{
//		u8 com_data = USART1->DR;
//		static u8 _data_len = 0,_data_cnt = 0;
//		if(RxState==0&&com_data==0xAA)
//		{
//			RxState=1;
//			RxBuffer[0]=com_data;
//		}
//		else if(RxState==1&&com_data==0xAF)
//		{
//			RxState=2;
//			RxBuffer[1]=com_data;
//		}
//		else if(RxState==2&&com_data>0&&com_data<0XF1)
//		{
//			RxState=3;
//			RxBuffer[2]=com_data;
//		}
//		else if(RxState==3&&com_data<50)
//		{
//			RxState = 4;
//			RxBuffer[3]=com_data;
//			_data_len = com_data;
//			_data_cnt = 0;
//		}
//		else if(RxState==4&&_data_len>0)
//		{
//			_data_len--;
//			RxBuffer[4+_data_cnt++]=com_data;
//			if(_data_len==0)
//				RxState = 5;
//		}
//		else if(RxState==5)
//		{
//			RxState = 0;
//			RxBuffer[4+_data_cnt]=com_data;
//			Data_Receive_Anl(RxBuffer,_data_cnt+5);
//		}
//		else
//			RxState = 0;
//	}
}
//int fputc(int ch, FILE *f)

//{

//USART_SendData(USART1, (uint8_t) ch);// USART1 ???? USART2 ?

//while (!(USART1->SR & USART_FLAG_TXE));

//return (ch);

//}
