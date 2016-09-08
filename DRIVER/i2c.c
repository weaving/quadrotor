#include "i2c.h"
#include "sys.h"
#include "includes.h"
#define CLEAR_ADDR_BIT      I2C_ReadRegister(I2C2, I2C_Register_SR1);\
                            I2C_ReadRegister(I2C2, I2C_Register_SR2);\
														
														

#define SCL_H         GPIOB->BSRRL = GPIO_Pin_10
#define SCL_L         GPIOB->BSRRH  = GPIO_Pin_10 
   
#define SDA_H         GPIOB->BSRRL = GPIO_Pin_11
#define SDA_L         GPIOB->BSRRH  = GPIO_Pin_11

#define SCL_read      GPIOB->IDR  & GPIO_Pin_10
#define SDA_read      GPIOB->IDR  & GPIO_Pin_11

void I2C_Configuration(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure; 

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);//��i2c����

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);//I2C2_SCL ��Ӧ PB10
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);//I2C2_SDA ��Ӧ PB11

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xd0;  //MPU6050
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  //ack enable
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;  //100k

	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStructure);

	I2C_AcknowledgeConfig(I2C2, ENABLE);//����ûʲô���� ֮ǰ��������
}

//void I2C_Configuration(void)
//{
////	I2C_InitTypeDef  I2C_InitStructure;
//	GPIO_InitTypeDef  GPIO_InitStructure; 

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);//��i2c����

////	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);//I2C2_SCL ��Ӧ PB10
////	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);//I2C2_SDA ��Ӧ PB11

//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

////	I2C_DeInit(I2C2);
////	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
////	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
////	I2C_InitStructure.I2C_OwnAddress1 = 0xd0;  //MPU6050
////	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  //ack enable
////	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
////	I2C_InitStructure.I2C_ClockSpeed = 400000;  //100k

////	I2C_Cmd(I2C2, ENABLE);
////	I2C_Init(I2C2, &I2C_InitStructure);

////	I2C_AcknowledgeConfig(I2C2, ENABLE);//����ûʲô���� ֮ǰ��������
//}
void SDA_OUT(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;			
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_OUT;		  	// ��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 	// ����������50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);				 	    // ѡ��C�˿�
}
void SDA_IN(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;			
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IN;  	// ��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);				     	// ѡ��C�˿�
}
void I2C_delay(void)
{
		
   u8 i=0; //�Ż��ٶ�
   while(i) 
   { 
     i--; 
   }  
}
void I2C_Start(void)
{	
    SCL_L;		 //����ʱ����
	I2C_delay(); //��ʱ
    SDA_OUT();     //sda�����
    SDA_H;	     //����������
	SCL_H;	     //����ʱ����
	I2C_delay(); //��ʱ
	SDA_L;		 //�����½���
	I2C_delay();	//��ʱ5us
	SCL_L;		 //����ʱ����

}

void I2C_Stop(void)
{
    SDA_OUT();//sda�����
    SCL_L;
    I2C_delay();             //��ʱ
    SDA_L;                    //����������
	I2C_delay();             //��ʱ
    SCL_H;                    //����ʱ����
    I2C_delay();             //��ʱ
    SDA_H;                   //����������
    I2C_delay();            //��ʱ
}

void I2C_SendACK(uint8_t ack)
{
	SCL_L;                  //����ʱ����
	SDA_OUT();
    I2C_delay();          //��ʱ
    if(ack)
	{
	 SDA_H; 				   // �޴��ź�
	}
	else
	{
	 SDA_L;				   //дӦ���ź�
	}
    I2C_delay();           //��ʱ                
    SCL_H;                  //����ʱ����
    I2C_delay();            //��ʱ
    SCL_L;                  //����ʱ����
    I2C_delay();            //��ʱ
}

uint8_t I2C_RecvACK(void)
{
    uint8_t i = 0;

	SCL_L;                   //����ʱ����
    I2C_delay();            //��ʱ
	SDA_H;
	I2C_delay();             //��ʱ
    SCL_H;                   //����ʱ����
    I2C_delay();             //��ʱ
   	SDA_IN();               //SDA����Ϊ���� 
    I2C_delay();            //��ʱ
	while(SDA_read)
	{
	   i++;
	   if(i>250)
	   {
	   	I2C_Stop();
		return 1;
	   }
	}
    SCL_L;                  //����ʱ����
    I2C_delay();           //��ʱ
    return 0;
}

void I2C_SendByte(uint8_t dat)
{
    uint8_t i;
	SDA_OUT();
	SCL_L;				//����ʱ���� 
    for (i=0; i<8; i++)         //8λ������
    {    
	   // DelayuS(1);             //��ʱ
		if(dat&0x80)	  //�Ƴ����ݵ����λ
		{
		 SDA_H;				  //�����ݿ�
		}
		else
		{
	     SDA_L;
		} 
		dat<<=1;
		SCL_H;				//����ʱ����
	    I2C_delay();            //��ʱ
		SCL_L;				//����ʱ����
	    I2C_delay();            //��ʱ
		
    }
   // while(I2C_RecvACK()==1);  
}

uint8_t I2C_RecvByte(void)
{
    uint8_t i;
    uint8_t dat = 0;
    //SDA_H;                      //ʹ���ڲ�����,׼����ȡ����,
	SDA_IN();//SDA����Ϊ����
    for (i=0; i<8; i++)         //8λ������
    {
		SCL_L;                   //����ʱ����
    	I2C_delay();             //��ʱ
		SCL_H;                   //����ʱ����
		dat <<= 1;
	    if(SDA_read)
		{
		 dat++;
		}
        I2C_delay();            //��ʱ
	}
    return dat;
}

//void I2C_ByteWrite(u8 SlaveAddress, u8 REG_Address, u8 REG_data)		     //void
//{
//    I2C_Start();
//	I2C_SendByte(SlaveAddress);
//	I2C_RecvACK();
//	I2C_SendByte(REG_Address);
//	I2C_RecvACK();
//	I2C_SendByte(REG_data);
//	I2C_RecvACK();  
//    I2C_Stop(); 
//}

//u8 I2C_ByteRead(u8 SlaveAddress,u8 REG_Address)
//{   
//	u8 REG_data;
//	I2C_Start();
//	I2C_SendByte(SlaveAddress);
//	I2C_RecvACK();
//	I2C_SendByte(REG_Address);
//	I2C_RecvACK();
//	I2C_Start();
//	I2C_SendByte(SlaveAddress+1);
//	I2C_RecvACK();
//	REG_data=I2C_RecvByte();
//	I2C_SendACK(1);
//	I2C_Stop();
//	return REG_data;
//}

void MS5611_SendCommand(u8 SlaveAddress,u8 REG_data)
{
    I2C_Start();
	I2C_SendByte(SlaveAddress);
	I2C_RecvACK();
	I2C_SendByte(REG_data);
	I2C_RecvACK();
	I2C_Stop();
}

void IIC_read_bytes(u8 SlaveAddress,u8 REG_Address,u8 *RxBuffer,u8 RxLenth)
{    	
	I2C_Start();
	I2C_SendByte(SlaveAddress);
	I2C_RecvACK();
	I2C_SendByte(REG_Address);
	I2C_RecvACK();
   	I2C_Stop();

	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_RecvACK();
	while(RxLenth)
    {
      *RxBuffer = I2C_RecvByte();
      if(RxLenth == 1) I2C_SendACK(1);
      else I2C_SendACK(0);
      RxBuffer++;
      RxLenth--;
    }
	I2C_Stop();
}


void I2C_ByteWrite(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
	OS_CPU_SR cpu_sr=0;

	OS_ENTER_CRITICAL();	
	I2C_GenerateSTART(I2C2,ENABLE);

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2,SlaveAddress,I2C_Direction_Transmitter);

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2,REG_Address);

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C2,REG_data);

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C2,ENABLE);

	OS_EXIT_CRITICAL();	

}


uint8_t I2C_ByteRead(uint8_t addr , uint8_t REG_Address)
{
	OS_CPU_SR cpu_sr=0;

	uint8_t REG_data;
	
	OS_ENTER_CRITICAL();	

	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C2,ENABLE);//????

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2,addr,I2C_Direction_Transmitter);//??????+???

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//

	I2C_Cmd(I2C2,ENABLE);

	I2C_SendData(I2C2,REG_Address);//????????,?0??

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C2,ENABLE);//????

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2,addr,I2C_Direction_Receiver);//??????+???

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	I2C_AcknowledgeConfig(I2C2,DISABLE);

	I2C_GenerateSTOP(I2C2,ENABLE);

	while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));

	REG_data=I2C_ReceiveData(I2C2);//???????

	OS_EXIT_CRITICAL();	

	return REG_data;

}

uint8_t* I2C_GetData(uint8_t addr,uint8_t REG_data, uint8_t length, uint8_t*  Rece_Data)
{
	OS_CPU_SR cpu_sr=0;
	u8 i=length;
	u8 j=0;
	OS_ENTER_CRITICAL();	
	I2C_AcknowledgeConfig(I2C2,ENABLE);

	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C2,ENABLE);//????

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2,addr,I2C_Direction_Transmitter);//??????+???

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));//
	
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_AF));
	
	I2C_Cmd(I2C2,ENABLE);

	I2C_SendData(I2C2,REG_data);//read ACCEL_XOUT_H

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_AF));

	I2C_GenerateSTART(I2C2,ENABLE);//????

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2,addr,I2C_Direction_Receiver);//??????+???

	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	
	
	  /* While there is data to be read */
  while(i)
  {
    if(i == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C2, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C2, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      Rece_Data[j] = I2C_ReceiveData(I2C2);

      /* Point to the next location where the byte read will be saved */
      j++;

      /* Decrement the read bytes counter */
      i--;
    }
  }
  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C2, ENABLE);
	OS_EXIT_CRITICAL();	
	return Rece_Data;
	
}
