#include "i2c.h"
#include "MPU6050.h"
#include "delay.h"  
#include "stm32f4xx_i2c.h"  
#include "sys.h"
#include "includes.h"
//#include "flash_eeprom.h"
#define M_PI  (float)3.1415926535

S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//��Ư
u8				GYRO_OFFSET_OK = 0;
u8				ACC_OFFSET_OK = 0;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡ

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_Dataanl(void)
*��������:	    ÿ1ms�ɼ�һ�����ݣ���ͣ�Ĳɣ���һ�ε�ʱ��ؽ�����Ư�˲�
*******************************************************************************/
void MPU6050_Dataanl(void)
{
		MPU6050_ACC_LAST.X=	GetData_MPU6050(ACCEL_XOUT_H) - ACC_OFFSET.X;
		MPU6050_ACC_LAST.Y= GetData_MPU6050(ACCEL_YOUT_H) - ACC_OFFSET.Y;
		MPU6050_ACC_LAST.Z= GetData_MPU6050(ACCEL_ZOUT_H) ;
	  /*
		���Լ��������� -3000 ����0g, Ҳ����ACC_OFFSET.Z=-3000, 14000����1g,  ������ҪУ׼��
		���ҵ���������������2g����32768��ֵʱ�����ʱ�����ֱ�� MPU6050_ACC_LAST.Z- ACC_OFFSET.Z;	
	  �ᵼ��16�������������ֵ����������
	  */
		if( MPU6050_ACC_LAST.Z > GYRO_OFFSET.Y+32768)  MPU6050_ACC_LAST.Z =ACC_OFFSET.Y+32768;
		else if( MPU6050_ACC_LAST.Z < GYRO_OFFSET.Y-32768)  MPU6050_ACC_LAST.Z =ACC_OFFSET.Y-32768;
		else MPU6050_ACC_LAST.Z = MPU6050_ACC_LAST.Z- ACC_OFFSET.Z;		
		//�����¶�ADC
		//������У׼
		MPU6050_GYRO_LAST.X=GetData_MPU6050(GYRO_XOUT_H) - GYRO_OFFSET.X;
		MPU6050_GYRO_LAST.Y=GetData_MPU6050(GYRO_YOUT_H) - GYRO_OFFSET.Y;
		MPU6050_GYRO_LAST.Z=GetData_MPU6050(GYRO_ZOUT_H)- GYRO_OFFSET.Z;	 
		
		if(!GYRO_OFFSET_OK)//�������ǽ�����㲹��,ȡ200��ƽ��ֵ GYRO_OFFSET_OK ACC_OFFSET_OK
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
				
				GPIO_ResetBits(GPIOA,GPIO_Pin_6 );//���øߣ�����
				//Flash_SaveOFFSET();//��������
				return;
			}
			cnt_g++;
		}
		if(!ACC_OFFSET_OK)//�Լ��ٶȵ���㲹����ȡ200�ν���ƽ��
		{
			static int32_t	tempax=0, tempay=0, tempaz=0;
			static uint16_t cnt_a=0;
			if(cnt_a==0)
			{
				ACC_OFFSET.X = 0;
				ACC_OFFSET.Y = 0;
				ACC_OFFSET.Z = 0;
				tempax = 0;
				tempay = 0;
				tempaz = 0;
				cnt_a = 1;
				return;
			}
			tempax+= MPU6050_ACC_LAST.X;
			tempay+= MPU6050_ACC_LAST.Y;
			tempaz+= MPU6050_ACC_LAST.Z;
			if(cnt_a==200)
			{
				ACC_OFFSET.X=tempax/cnt_a;
				ACC_OFFSET.Y=tempay/cnt_a;
				ACC_OFFSET.Z=tempaz/cnt_a-16384;
				cnt_a = 0;
				ACC_OFFSET_OK = 1;
				//Flash_SaveOFFSET();//��������
				return;
			}
			cnt_a++;		
		}
}

void MPU6050_Init()
{
	I2C_Configuration();
	delay_ms(100);
	I2C_ByteWrite(MPU6050_I2C_Addr,PWR_MGMT_1,0X80);
	delay_ms(10);
	I2C_ByteWrite(MPU6050_I2C_Addr,PWR_MGMT_1,0X00);
	I2C_ByteWrite(MPU6050_I2C_Addr,SMPLRT_DIV,0x07);//125hz 1k/1+0x03
	I2C_ByteWrite(MPU6050_I2C_Addr,MPU6050_CONFIG,0x06);
	delay_ms(1);
	I2C_ByteWrite(MPU6050_I2C_Addr,GYRO_CONFIG,0x18);// +-2000
	delay_ms(1);
	I2C_ByteWrite(MPU6050_I2C_Addr,ACCEL_CONFIG,0x0<<3);//+-2G
	/* mpu6050 xcl xda bypass for i2c */
	I2C_ByteWrite(MPU6050_I2C_Addr,I2C_MST_CTRL,0x00);	
	I2C_ByteWrite(MPU6050_I2C_Addr,INT_PIN_CFG,0x02);
	
//	I2C_ByteWrite(MPU6050_I2C_Addr,USER_CTRL,0x20);
//	I2C_ByteWrite(MPU6050_I2C_Addr,I2C_MST_CTRL, 0x0D);
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x25, 0x80|HMC5883_I2C_Addr);
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x26, MAG_DATA_REGISTER );
//	I2C_ByteWrite(MPU6050_I2C_Addr, 0x27, 0x86); 

}

uint16_t GetData_MPU6050(uint8_t REG_Address)
{
	uint8_t H,L;
	H=I2C_ByteRead(MPU6050_I2C_Addr,REG_Address);
	L=I2C_ByteRead(MPU6050_I2C_Addr,REG_Address+1);
	return (H<<8)|L; 
}
