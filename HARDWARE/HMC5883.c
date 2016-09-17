#include "HMC5883.h"
#include "imu.h"
#include "i2c.h"
#include "math.h"
#include "data_transfer.h"
#include "flash_eeprom.h"
#include "arm_math.h"
#include "CONTROL.h"
S_INT16_XYZ HMC5883L_LAST;
#define HMC5883_Buf_Size 10
//HMC5883L
S_INT16_XYZ HMC5883L_LAST,AVG_MAG,MOTO1_MAG,MOTO_MAG,MOTO1_MAG,MOTO2_MAG,MOTO3_MAG,MOTO4_MAG;
S_INT16_XYZ	Max,Min;
S_INT16_XYZ HMC5883L_OFFSET;
S_FLOAT_XYZ HMC5883L_Gain;
u8		    MAG_OFFSET_OK = 1;
float H_YAW;
int8_t  HMC5883_Buf_index = 0;
int16_t  HMC5883_FIFO[3][HMC5883_Buf_Size]; //磁力计滤波
															  

uint16_t GetData_HMC5883(uint8_t REG_Address)
{
	uint8_t H,L;
	H=I2C_ByteRead(HMC5883_I2C_Addr,REG_Address);
	L=I2C_ByteRead(HMC5883_I2C_Addr,REG_Address+1);
	return (H<<8)|L;  
}

void HMC5883_Init()
{
	I2C_ByteWrite(HMC5883_I2C_Addr,0x00, 0x78);	 //8HZ滤波 75HZ输出
	I2C_ByteWrite(HMC5883_I2C_Addr,0x01, 0x20);	 //默认
	I2C_ByteWrite(HMC5883_I2C_Addr,0x02,0x00);
}

void HMC5883L_Dataanl(void)
{	           
	unsigned char i ;
	int32_t sum=0;
	if (MAG_OFFSET_OK == 0)  //校正地磁传感器
	{
		HMC5883L_OFFSET.X=0;
		HMC5883L_OFFSET.Y=0;
		HMC5883L_OFFSET.Z=0;
		HMC5883L_Gain.X=1; 
		HMC5883L_Gain.Y=1;
		HMC5883L_Gain.Z=1;	
	}     

	HMC5883L_LAST.X = (float)(HMC5883L_Gain.X * (int16_t)(GetData_HMC5883(0x03) - HMC5883L_OFFSET.X));
	HMC5883L_LAST.Z = (float)(HMC5883L_Gain.Z * (int16_t)(GetData_HMC5883(0x05) - HMC5883L_OFFSET.Z));
	HMC5883L_LAST.Y = (float)(HMC5883L_Gain.Y * (int16_t)(GetData_HMC5883(0x07) - HMC5883L_OFFSET.Y));

	GetMotoMag();
	
	HMC5883_FIFO[0][HMC5883_Buf_index] = HMC5883L_LAST.X - MOTO_MAG.X;
	HMC5883_FIFO[1][HMC5883_Buf_index] = HMC5883L_LAST.Y - MOTO_MAG.Y;
	HMC5883_FIFO[2][HMC5883_Buf_index] = HMC5883L_LAST.Z - MOTO_MAG.Z;

	HMC5883_Buf_index = (HMC5883_Buf_index + 1) % HMC5883_Buf_Size;
	sum=0;
	for(i=0;i<HMC5883_Buf_Size;i++)
	{	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	AVG_MAG.X=sum/HMC5883_Buf_Size;	//将平均值更新

	sum=0;
	for(i=0;i<HMC5883_Buf_Size;i++)
	{
   		sum+=HMC5883_FIFO[1][i];
	}
	AVG_MAG.Y=sum/HMC5883_Buf_Size;

	sum=0;
	for(i=0;i<HMC5883_Buf_Size;i++)
	{
   		sum+=HMC5883_FIFO[2][i];
	}
	AVG_MAG.Z=sum/HMC5883_Buf_Size;

	AVG_MAG.X = AVG_MAG.X;
	AVG_MAG.Y = AVG_MAG.Y;
	AVG_MAG.Z = AVG_MAG.Z;

//	H_YAW = HMC5883L_Calculate(); //在IMU里已经计算 所以这里不用

	if (MAG_OFFSET_OK == 0)  //校正地磁传感器
	{
		HMC5883L_Correction();
	} 

}


void HMC5883L_Correction(void)
{
	static uint16_t cnt_h=0;
	HMC5883L_GetMaxMin(HMC5883L_LAST.X,HMC5883L_LAST.Y,HMC5883L_LAST.Z);
	cnt_h++;
	if(cnt_h %30==0)
	GPIO_ToggleBits(GPIOA, GPIO_Pin_7);
	if (cnt_h==5000)   //60秒校正时间
	{
		HMC5883L_GetOFFSET();

		Flash_SaveOFFSET();
		MAG_OFFSET_OK = 1;
		cnt_h = 0;	
		GPIO_ResetBits(GPIOA,GPIO_Pin_6 | GPIO_Pin_7);//设置高，灯灭

	}		
}

void HMC5883L_GetMaxMin(int16_t X,int16_t Y,int16_t Z)
{
	if(X<Min.X) Min.X=X; 
	if(X>Max.X) Max.X=X;
	if(Y<Min.Y) Min.Y=Y; 
	if(Y>Max.Y) Max.Y=Y;
	if(Z<Min.Z) Min.Z=Z; 
	if(Z>Max.Z) Max.Z=Z;	

}

void HMC5883L_GetOFFSET(void)
{
	HMC5883L_OFFSET.X = (Min.X + Max.X) / 2;
	HMC5883L_OFFSET.Y = (Min.Y + Max.Y) / 2;
	HMC5883L_OFFSET.Z = (Min.Z + Max.Z) / 2;

	HMC5883L_Gain.X = 1;
	HMC5883L_Gain.Y = (float)(Max.X - Min.X) / (Max.Y - Min.Y);
	HMC5883L_Gain.Z = (float)(Max.X - Min.X) / (Max.Z - Min.Z);

}

float HMC5883L_Calculate(void)
{
	int mx,my,mz;
	float h_yaw;
		
	mx=AVG_MAG.X;
	my=AVG_MAG.Y;
	mz=AVG_MAG.Z;

	h_yaw = atan2(my*cos(-Roll) + mx*sin(-Roll)*sin(-Pitch) - mz*sin(-Roll)*cos(-Pitch), mx*cos(-Pitch)+mz*sin(-Pitch))*180/3.14159265;
//	h_yaw = my*arm_cos_f32(-Roll) + mx*arm_sin_f32(-Roll)*arm_sin_f32(-Pitch) - mz*arm_sin_f32(-Roll)*arm_cos_f32(-Pitch);

	if(h_yaw<0) h_yaw+=360;	

	return h_yaw;
}

void GetMotoMag(void) //动态估计电机磁场
{
	float Gain1,Gain2,Gain3,Gain4;

	Gain1 = ((float)(RCTarget.Throttle-1000)/1000);
	Gain2 = ((float)(RCTarget.Throttle-1000)/1000);
	Gain3 = ((float)(RCTarget.Throttle-1000)/1000);
	Gain4 = ((float)(RCTarget.Throttle-1000)/1000);

	MOTO1_MAG.X	= (int)(Gain1*Gain1*(-30));
	MOTO1_MAG.Y	= (int)(Gain1*Gain1*(8));
	MOTO1_MAG.Z = (int)(Gain1*Gain1*(187));

	MOTO2_MAG.X	= (int)(Gain2*Gain2*(-112));
//	MOTO2_MAG.Y	= (int)(Gain2*Gain2*(120));
	MOTO2_MAG.Y	= (int)(Gain2*Gain2*(105)-6*Gain2+3);
	MOTO2_MAG.Z = (int)(Gain2*Gain2*(-152));

	MOTO3_MAG.X	= (int)(Gain3*Gain3*(100));
//	MOTO3_MAG.Y	= (int)(Gain3*Gain3*(150));
	MOTO3_MAG.Y = (int) 95*Gain3*Gain3+36*Gain3+3;
	MOTO3_MAG.Z = (int)(Gain3*Gain3*(-87));

	MOTO4_MAG.X	= (int)(Gain4*Gain4*120);
//	MOTO4_MAG.Y	= (int)(Gain4*Gain4*(120));
	MOTO4_MAG.Y	= (int)(Gain4*Gain4*(119)+Gain4*20+1);	
	MOTO4_MAG.Z = (int)(Gain4*Gain4*(-80));
	
	MOTO_MAG.X = MOTO1_MAG.X + MOTO2_MAG.X + MOTO3_MAG.X + MOTO4_MAG.X;
	MOTO_MAG.Y = MOTO1_MAG.Y + MOTO2_MAG.Y + MOTO3_MAG.Y + MOTO4_MAG.Y;
	MOTO_MAG.Z = MOTO1_MAG.Z + MOTO2_MAG.Z + MOTO3_MAG.Z + MOTO4_MAG.Z;
}
