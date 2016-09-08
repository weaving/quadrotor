#include "i2c.h"
#include "MPU6050.h"
#include "delay.h"  
#include "stm32f4xx_i2c.h"  
#include "sys.h"
#include "includes.h"
//#include "flash_eeprom.h"
#define M_PI  (float)3.1415926535
	
uint8_t buffer[14];
int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;

S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//ÁãÆ¯
u8				GYRO_OFFSET_OK = 0;
u8				ACC_OFFSET_OK = 0;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//×îÐÂÒ»´Î¶ÁÈ¡Ö
uint8_t MPU6050_ACCEL_Data[6],MPU6050_GYRO_Data[6];

/**************************ÊµÏÖº¯Êý********************************************
*º¯ÊýÔ­ÐÍ:		void MPU6050_Dataanl(void)
*¹¦¡¡¡¡ÄÜ:	    Ã¿1ms²É¼¯Ò»´ÎÊý¾Ý£¬²»Í£µÄ²É£¬µÚÒ»´ÎµÄÊ±ºò»Ø½øÐÐÁãÆ¯ÂË²¨
*******************************************************************************/
void MPU6050_Dataanl(void)
{
//	if(MPU6050_IS_RDY)
//		{ 
		//MPU6050Êý¾Ý¶ÁÈ¡
//	  I2C_GetData(MPU6050_I2C_Addr,ACCEL_XOUT_H,6,MPU6050_ACCEL_Data);
//	  I2C_GetData(MPU6050_I2C_Addr,GYRO_XOUT_H,6,MPU6050_GYRO_Data);
     
		//¼ÓËÙ¶È¼ÆÐ£×¼ 
//		MPU6050_ACC_LAST.X=MPU6050_ACCEL_Data[0]<<8|MPU6050_ACCEL_Data[1] - ACC_OFFSET.X;
//		MPU6050_ACC_LAST.Y=MPU6050_ACCEL_Data[2]<<8|MPU6050_ACCEL_Data[3] - ACC_OFFSET.Y;
//		MPU6050_ACC_LAST.Z=MPU6050_ACCEL_Data[4]<<8|MPU6050_ACCEL_Data[5]; //- ACC_OFFSET.Z;
		MPU6050_ACC_LAST.X=	GetData_MPU6050(ACCEL_XOUT_H) - ACC_OFFSET.X;
		MPU6050_ACC_LAST.Y=GetData_MPU6050(ACCEL_YOUT_H) - ACC_OFFSET.Y;
		MPU6050_ACC_LAST.Z=GetData_MPU6050(ACCEL_ZOUT_H); //- ACC_OFFSET.Z;
		//Ìø¹ýÎÂ¶ÈADC
		//ÍÓÂÝÒÇÐ£×¼
//		MPU6050_GYRO_LAST.X=MPU6050_GYRO_Data[0]<<8|MPU6050_GYRO_Data[1] - GYRO_OFFSET.X;
//		MPU6050_GYRO_LAST.Y=MPU6050_GYRO_Data[2]<<8|MPU6050_GYRO_Data[3] - GYRO_OFFSET.Y;
//		MPU6050_GYRO_LAST.Z=MPU6050_GYRO_Data[4]<<8|MPU6050_GYRO_Data[5] - GYRO_OFFSET.Z;
		MPU6050_GYRO_LAST.X=GetData_MPU6050(GYRO_XOUT_H) - GYRO_OFFSET.X;
		MPU6050_GYRO_LAST.Y=GetData_MPU6050(GYRO_YOUT_H) - GYRO_OFFSET.Y;
		MPU6050_GYRO_LAST.Z=GetData_MPU6050(GYRO_ZOUT_H)- GYRO_OFFSET.Z;	 
		if(!GYRO_OFFSET_OK)//¶ÔÍÓÂÝÒÇ½øÐÐÁãµã²¹³¥,È¡200´ÎÆ½¾ùÖµ GYRO_OFFSET_OK ACC_OFFSET_OK
		{
			static int32_t	tempgx=0,tempgy=0,tempgz=0;
			static uint16_t cnt_g=0;
			if(cnt_g==0)
			{
				GYRO_OFFSET.X=0;
				GYRO_OFFSET.Y=0;
				GYRO_OFFSET.Z=0;
				tempgx = 0;
				tempgy = 0;
				tempgz = 0;
				cnt_g = 1;
				return;
			}
			tempgx+= MPU6050_GYRO_LAST.X;
			tempgy+= MPU6050_GYRO_LAST.Y;
			tempgz+= MPU6050_GYRO_LAST.Z;
			if(cnt_g==200)
			{
				GYRO_OFFSET.X=tempgx/cnt_g;
				GYRO_OFFSET.Y=tempgy/cnt_g;
				GYRO_OFFSET.Z=tempgz/cnt_g;
				cnt_g = 0;
				GYRO_OFFSET_OK = 1;
				
				GPIO_ResetBits(GPIOA,GPIO_Pin_6 );//ÉèÖÃ¸ß£¬µÆÃð
				//Flash_SaveOFFSET();//±£´æÊý¾Ý
				return;
			}
			cnt_g++;
		}
		if(!ACC_OFFSET_OK)//¶Ô¼ÓËÙ¶ÈµÄÁãµã²¹³¥£¬È¡200´Î½øÐÐÆ½¾ù
		{
			static int32_t	tempax=0,tempay=0;//,tempaz=0;
			static uint16_t cnt_a=0;
			if(cnt_a==0)
			{
				ACC_OFFSET.X = 0;
				ACC_OFFSET.Y = 0;
				ACC_OFFSET.Z = 0;
				tempax = 0;
				tempay = 0;
//				tempaz = 0;
				cnt_a = 1;
				return;
			}
			tempax+= MPU6050_ACC_LAST.X;
			tempay+= MPU6050_ACC_LAST.Y;
			//tempaz+= MPU6050_ACC_LAST.Z;
			if(cnt_a==200)
			{
				ACC_OFFSET.X=tempax/cnt_a;
				ACC_OFFSET.Y=tempay/cnt_a;
				//ACC_OFFSET.Z=tempaz/cnt_a;
				cnt_a = 0;
				ACC_OFFSET_OK = 1;
				//Flash_SaveOFFSET();//±£´æÊý¾Ý
				return;
			}
			cnt_a++;		
		}
//	}
//		MPU6050_IS_RDY=0;
}

/**************************ÊµÏÖº¯Êý********************************************
*º¯ÊýÔ­ÐÍ:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*¹¦¡¡¡¡ÄÜ:	    ½«ÐÂµÄADCÊý¾Ý¸üÐÂµ½ FIFOÊý×é£¬½øÐÐÂË²¨´¦Àí
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){	//FIFO ²Ù×÷
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//½«ÐÂµÄÊý¾Ý·ÅÖÃµ½ Êý¾ÝµÄ×îºóÃæ
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;
sum=0;
for(i=0;i<10;i++){	//Çóµ±Ç°Êý×éµÄºÏ£¬ÔÙÈ¡Æ½¾ùÖµ
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

void MPU6050_Init()
{
	I2C_Configuration();
	delay_ms(100);
	I2C_ByteWrite(MPU6050_I2C_Addr,PWR_MGMT_1,0X80);
	delay_ms(10);
	I2C_ByteWrite(MPU6050_I2C_Addr,PWR_MGMT_1,0X00);
	I2C_ByteWrite(MPU6050_I2C_Addr,SMPLRT_DIV,0x03);//125hz 1k/1+0x03
	I2C_ByteWrite(MPU6050_I2C_Addr,MPU6050_CONFIG,0x06);
	delay_ms(1);
	I2C_ByteWrite(MPU6050_I2C_Addr,GYRO_CONFIG,0x18);// +-2000
	delay_ms(1);
	I2C_ByteWrite(MPU6050_I2C_Addr,ACCEL_CONFIG,0x00);//+-2G
	/* mpu6050 xcl xda bypass for i2c */
	I2C_ByteWrite(MPU6050_I2C_Addr,I2C_MST_CTRL,0x00);	
	I2C_ByteWrite(MPU6050_I2C_Addr,INT_PIN_CFG,0x02);
	
//	I2C_ByteWrite(MPU6050_I2C_Addr,USER_CTRL,0x20);
//	I2C_ByteWrite(MPU6050_I2C_Addr,I2C_MST_CTRL, 0x0D);
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x25, 0x80|HMC5883_I2C_Addr);
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x26, MAG_DATA_REGISTER );
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x27, 0x86); 

	
//	//ÅäÖÃMPU6050 µÄÖÐ¶ÏÄ£Ê½ ºÍÖÐ¶ÏµçÆ½Ä£Ê½
//	I2C_ByteWrite(MPU6050_I2C_Addr,INT_PIN_CFG,0x30);
//	//¿ªÊý¾Ý×ª»»Íê³ÉÖÐ¶Ï
//	I2C_ByteWrite(MPU6050_I2C_Addr,INT_ENABLE,0x01);

//	MPU6050_EXTI_Configuration();
  
//	Gx_offset = Config.dGx_offset;
//	Gy_offset = Config.dGy_offset;
//	Gz_offset = Config.dGz_offset;		
	
	Gx_offset = 0;
	Gy_offset = 0;
	Gz_offset = 0;
}



uint16_t GetData_MPU6050(uint8_t REG_Address)
{
	uint8_t H,L;
	H=I2C_ByteRead(MPU6050_I2C_Addr,REG_Address);
	L=I2C_ByteRead(MPU6050_I2C_Addr,REG_Address+1);
	return (H<<8)|L;   //????
}



//void MS5611_RESET(void)
//{
//	I2C_GenerateSTART(I2C2,ENABLE);
//	I2C_Send7bitAddress(I2C2,MS561101BA_ADDR,I2C_Direction_Transmitter);

//// I2C_RecvACK();
//	I2C_SendData(I2C2,MS561101BA_RESET);
//// I2C_RecvACK();
//	I2C_GenerateSTOP(I2C2,ENABLE);
//}

//void MPU6050_EXTI_Configuration(void)
//{
//  EXTI_InitTypeDef   EXTI_InitStructure;
//  GPIO_InitTypeDef   GPIO_InitStructure;
//  NVIC_InitTypeDef   NVIC_InitStructure;

//  /* Enable GPIOE clock */
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	/* Enable AFIO clock */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
//  /* Configure PE.1 pin as input floating */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  /* Connect EXTI1 Line to PB.1 pin */
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource1);
//	
//  /* Configure EXTI1 line */
//  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//  EXTI_Init(&EXTI_InitStructure);
// 
//  /* Enable and set EXTI9_5 Interrupt to the lowest priority */
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//×î¸ßÓÅÏÈ¼¶
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

//  NVIC_Init(&NVIC_InitStructure);
//}



//void EXTI1_IRQHandler(void)
//{

//	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line1);
//		MPU6050_IS_RDY = 1;
//	}
//}

