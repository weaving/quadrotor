#include "data_transfer.h"
#include "stm32f4xx.h"
#include "UART.h"
#include "MPU6050.h"
#include "hmc5883.h"
#include "24l01.h"
#include "IMU.h"
#include "pid.h"
#include "flash_eeprom.h"
#include "wft_controller.h"

u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_GpsData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_UserData,Send_Votage,Send_MotoPwm;
u8 data_to_send[50];
u8 SensorUserd=0;
u8 data_to_send_NRF[51];
u8 data_to_send_test[51] ={10,0xaa,0xaa,0x02,0x16,02,10,0,20,0,30,0};
int16_t UserData[20];

#define DATA_TRANSFER_USE_NRF 1
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

void Nrf_Check_Event(void)
{
	u8 sta = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);
	NRF24L01_RX_Mode();
	if(sta & (1<<RX_DR))		   //接收中断查询
	{
		u8 rx_len,i;
		
		NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

		rx_len = NRF24L01_RXDATA[0];
		for( i=0;i<RX_PLOAD_WIDTH;i++)
		{
			NRF24L01_RXDATA[i] = NRF24L01_RXDATA[i+1];
		}
		
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
		Data_Receive_Anl(NRF24L01_RXDATA,rx_len);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
	}
	if(sta & (1<<TX_DS))
	{
	}
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}			
	}

	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
}
void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	//vs16 rc_value_temp;
	u8 i,sum = 0;
	static u8 flag = 0;
	for(i=0;i<(num-1);i++) sum += *(data_buf+i);

	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			ACC_OFFSET_OK = 0;
		if(*(data_buf+4)==0X02)
			GYRO_OFFSET_OK = 0;
		if(*(data_buf+4)==0X03)
		{
			if(flag)
			{	
				SensorUserd = 0;
				flag = 0;
			}		    
			else
			{
				SensorUserd = 1;
				flag = 1;
			}
		}
		if(*(data_buf+4)==0X04)
			MAG_OFFSET_OK = 0;
		if(*(data_buf+4)==0X05)	
			;
//			Alt_Offset_Pa = 0;
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			Send_PID1 = 1;
			Send_PID2 = 1;
			Send_PID3 = 1;
		}
		if(*(data_buf+4)==0X02)
			Send_Offset = 1;
	}
	if(*(data_buf+2)==0X10)								//PID1
	{
			RollRate.Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			RollRate.Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			RollRate.Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;

			PitchRate.Kp = RollRate.Kp;
			PitchRate.Ki = RollRate.Ki;
			PitchRate.Kd = RollRate.Kd;
			Stabilize_Roll.Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			Stabilize_Roll.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			Stabilize_Roll.Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			Stabilize_Pitch.Kp = Stabilize_Roll.Kp;
			Stabilize_Pitch.Ki = Stabilize_Roll.Ki;
			Stabilize_Pitch.Kd = Stabilize_Roll.Kd;
		
			Position_Hold.Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			Position_Hold.Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			Position_Hold.Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		/*
			Stabilize_Yaw.Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/10;
			Stabilize_Yaw.Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			Stabilize_Yaw.Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/10;
			*/
			YawRate.Kp = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			YawRate.Ki = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			YawRate.Kd = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			
			Send_PID1 = 1;
			Data_Send_Check(sum);
//			for(i=0;i<100;i++) Data_Send_Check(sum);
			//Flash_SavePID();
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
			AutoHigh_THR.Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			AutoHigh_THR.Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			AutoHigh_THR.Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;

			Position_Speed.Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			Position_Speed.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			Position_Speed.Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			/*
			RollAccel.Kp = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			RollAccel.Ki = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			RollAccel.Kd = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/10;
			*/
			PitchAccel.Kp = RollAccel.Kp;
			PitchAccel.Ki = RollAccel.Ki;
			PitchAccel.Kd = RollAccel.Kd;

			Send_PID2 = 1;
			Data_Send_Check(sum);
//			for(i=0;i<100;i++) Respond_PID2();
			//Flash_SavePID();
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
			Climb.Kp = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			Climb.Ki = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			Climb.Kd = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			Send_PID3 = 1;
			Data_Send_Check(sum);
//			for(i=0;i<100;i++) Respond_PID3();
//			Flash_SavePID();
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
//			PID_PID_5.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_5.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_5.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_6.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_6.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_6.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_7.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_7.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_7.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
//			PID_PID_8.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_8.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_8.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_9.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_9.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_9.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_10.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_10.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_10.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
//			PID_PID_11.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_11.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_11.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_PID_12.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PID_12.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_PID_12.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{
			AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			Accel_Offset_Ok = 0;
	}	

	
/////////////////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0x03)								//判断功能字,=0x8a,为遥控数据
	{
//		u8 _cnt = 4;
//		#ifdef	RC_USE_DEADBAND
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.THROTTLE = RC_MIDDLE;
//			else
//				Rc_Data.THROTTLE = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.YAW = RC_MIDDLE;
//			else
//				Rc_Data.YAW = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.ROLL = RC_MIDDLE;
//			else
//				Rc_Data.ROLL = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.PITCH = RC_MIDDLE;
//			else
//				Rc_Data.PITCH = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX1 = RC_MIDDLE;
//			else
//				Rc_Data.AUX1 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX2 = RC_MIDDLE;
//			else
//				Rc_Data.AUX2 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX3 = RC_MIDDLE;
//			else
//				Rc_Data.AUX3 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX4 = RC_MIDDLE;
//			else
//				Rc_Data.AUX4 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX5 = RC_MIDDLE;
//			else
//				Rc_Data.AUX5 = rc_value_temp;
//			rc_value_temp = (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			if((RC_MIDDLE-RC_DEADBAND) < rc_value_temp && rc_value_temp < (RC_MIDDLE+RC_DEADBAND))
//				Rc_Data.AUX6 = RC_MIDDLE;
//			else
//				Rc_Data.AUX6 = rc_value_temp;
//		#else
//			Rc_Data.THROTTLE 	= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.YAW				= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.ROLL			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.PITCH			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX1			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX2			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX3			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX4			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX5			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//			Rc_Data.AUX6			= (vs16)(*(data_buf+_cnt)<<8)|*(data_buf+_cnt+1);
//			_cnt += 2;
//		#endif
//		Rc_DataCal();
	}
}

void Send_Data(void)
{
	u8 sta;
#ifdef DATA_TRANSFER_USE_NRF
	Nrf_Check_Event();
	sta = Nrf_Get_FIFOSta();
	if((sta & (1<<5))==1)
		return;
#endif
	if(Lock_dataTransfer==1)
	{
		if(Send_Status)
		{
			Send_Status = 0;
			Data_Send_Status();

//			Data_Send_UserData();

		}
		else if(Send_GpsData)
		{
			Send_GpsData = 0;
			//Data_Send_GpsData();
		}
		else if(Send_Senser)
		{
			Send_Senser = 0;
			Data_Send_Senser();
		}
		else if(Send_PID1)
		{
			Send_PID1 = 0;
			Data_Send_PID1();
		}
		else if(Send_PID2)
		{
			Send_PID2 = 0;
			Data_Send_PID2();
		}
		else if(Send_PID3)
		{
			Send_PID3 = 0;
			Data_Send_PID3();
		}
		else if(Send_RCData)
		{
			Send_RCData = 0;
			//Data_Send_RCData();
		}
		else if(Send_Offset)
		{
			Send_Offset = 0;
			Data_Send_OFFSET();
		}
		else if(Send_Votage)
		{
			Send_Votage = 0;
			//Data_Send_Votage();
		} 
		else if(Send_MotoPwm)
		{
			Send_MotoPwm = 0;
			//Data_Send_MotoPWM();
		}
		else if(Send_UserData)
		{
			Send_UserData = 0;
			Data_Send_UserData();
		}
	}
}

void Data_Send_Senser(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(MPU6050_ACC_LAST.X);
	data_to_send[_cnt++]=BYTE0(MPU6050_ACC_LAST.X);
	data_to_send[_cnt++]=BYTE1(MPU6050_ACC_LAST.Y);
	data_to_send[_cnt++]=BYTE0(MPU6050_ACC_LAST.Y);
	data_to_send[_cnt++]=BYTE1(MPU6050_ACC_LAST.Z);
	data_to_send[_cnt++]=BYTE0(MPU6050_ACC_LAST.Z);
	data_to_send[_cnt++]=BYTE1(MPU6050_GYRO_LAST.X);
	data_to_send[_cnt++]=BYTE0(MPU6050_GYRO_LAST.X);
	data_to_send[_cnt++]=BYTE1(MPU6050_GYRO_LAST.Y);
	data_to_send[_cnt++]=BYTE0(MPU6050_GYRO_LAST.Y);
	data_to_send[_cnt++]=BYTE1(MPU6050_GYRO_LAST.Z);
	data_to_send[_cnt++]=BYTE0(MPU6050_GYRO_LAST.Z);
	

	
	data_to_send[_cnt++]=BYTE1(AVG_MAG.X);
	data_to_send[_cnt++]=BYTE0(AVG_MAG.X);
	data_to_send[_cnt++]=BYTE1(AVG_MAG.Y);
	data_to_send[_cnt++]=BYTE0(AVG_MAG.Y);
	data_to_send[_cnt++]=BYTE1(AVG_MAG.Z);
	data_to_send[_cnt++]=BYTE0(AVG_MAG.Z);
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

void Data_Send_Status(void)
{
	u8 _cnt=0;
	u8 sum = 0;
	u8 i;
	vs16 _temp;
	vs32 _temp2 = Alt;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(Q_ANGLE.X*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Q_ANGLE.Y*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Q_ANGLE.Z*100);
	//_temp = (int)(Mag_Heading*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
//printf("tmp : %8.3f %8.3f %8.3f\r\n",Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z);
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	//if(Rc_C.ARMED==0)			
		data_to_send[_cnt++]=0xA0;	//锁定
	//else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;
	
	data_to_send[3] = _cnt-4;

	for( i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

void Data_Send_UserData(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	
	_temp = UserData[0];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[1];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[2];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[3];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[4];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[5];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[6];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[7];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[8];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[9];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[10];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = UserData[11];
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

void Respond_PID1(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	_temp = RollRate.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Kp * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}
void Respond_PID2(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	_temp = AutoHigh_THR.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AutoHigh_THR.Ki * 10000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AutoHigh_THR.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	/*
	_temp = RollAccel.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollAccel.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollAccel.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	*/
	_temp = YawRate.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}
void Respond_PID3(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x12;
	data_to_send[_cnt++]=0;
	
	_temp = Climb.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Climb.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Climb.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}
void Data_Send_Check(u16 check)
{
	u8 sum = 0,i;
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF0;
	data_to_send[_cnt++]=3;
	data_to_send[_cnt++]=0xBA;
	
	data_to_send[_cnt++]=BYTE1(check);
	data_to_send[_cnt++]=BYTE0(check);
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

void Data_Send_PID1(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	_temp = RollRate.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollRate.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Kp * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Stabilize_Roll.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Hold.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

void Data_Send_PID2(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;
	
	_temp = AutoHigh_THR.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AutoHigh_THR.Ki * 10000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AutoHigh_THR.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Ki * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Position_Speed.Kd * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	/*
	_temp = RollAccel.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollAccel.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = RollAccel.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	*/
	_temp = YawRate.Kp * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = YawRate.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}
void Data_Send_PID3(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x12;
	data_to_send[_cnt++]=0;
	
	_temp = Climb.Kp * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Climb.Ki * 100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Climb.Kd * 10;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

//void Data_Send_GpsData(void)
//{
//	u8 i,_cnt=0,sum = 0;
//	vs16 _temp;
//	int32_t _temp32;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x04;				  
//	data_to_send[_cnt++]=0;
//	_temp32 = (int)(Latitude_GPS[0]*10000000) + (int)(Latitude_GPS[1]*10000000);
//	data_to_send[_cnt++]=BYTE3(_temp32);
//	data_to_send[_cnt++]=BYTE2(_temp32);
//	data_to_send[_cnt++]=BYTE1(_temp32);
//	data_to_send[_cnt++]=BYTE0(_temp32);
//	_temp32 = (int)(Longitude_GPS[0]*10000000) + (int)(Longitude_GPS[1]*10000000);
//	data_to_send[_cnt++]=BYTE3(_temp32);
//	data_to_send[_cnt++]=BYTE2(_temp32);
//	data_to_send[_cnt++]=BYTE1(_temp32);
//	data_to_send[_cnt++]=BYTE0(_temp32);
//	data_to_send[_cnt++]=BYTE3(GPS_Altitude);
//	data_to_send[_cnt++]=BYTE2(GPS_Altitude);
//	data_to_send[_cnt++]=BYTE1(GPS_Altitude);
//	data_to_send[_cnt++]=BYTE0(GPS_Altitude);
//	data_to_send[_cnt++]=BYTE3(Speed_GPS);
//	data_to_send[_cnt++]=BYTE2(Speed_GPS);
//	data_to_send[_cnt++]=BYTE1(Speed_GPS);
//	data_to_send[_cnt++]=BYTE0(Speed_GPS);
//	_temp = (int)(GPS_mLat_Distance*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(GPS_mLon_Distance*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	data_to_send[_cnt++]=GPS_Status;
//	data_to_send[_cnt++]=GPS_STA_Num;
//	
//	data_to_send[3] = _cnt-4;
//	
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF24L01_TxPacket(data_to_send_NRF);
//#endif
//}

void Data_Send_Votage(void)
{
	u8 i,_cnt=0,sum = 0;
	u16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	_temp=3700;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=3700;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp=3700;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}

//void Data_Send_MotoPWM(void)
//{
//	u8 i,_cnt=0,sum = 0;
//	u16 _temp;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x06;
//	data_to_send[_cnt++]=0;
//	_temp=MOTO1_THR-1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp=MOTO2_THR-1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp=MOTO3_THR-1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp=MOTO4_THR-1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp=0;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF24L01_TxPacket(data_to_send_NRF);
//#endif
//}
void Data_Send_OFFSET(void)
{
	u8 i,_cnt=0,sum = 0;
	vs16 _temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x16;
	data_to_send[_cnt++]=0;
    _temp = AngleOffset_Rol*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = AngleOffset_Pit*1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	for( i=0;i<_cnt;i++)
	{
		data_to_send_NRF[i+1] = data_to_send[i];
	}
	data_to_send_NRF[0] = _cnt;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send_NRF,_cnt);
#else
	NRF24L01_TxPacket(data_to_send_NRF);
#endif
}
