#include "math.h"
#include "PID.h"
//#include "pwm_capture.h"
#include "flash_eeprom.h"
#include "MOTO.h"
#include "CONTROL.h"
#include "hmc5883.h"
#include "IMU.h"
//#include "ms5611.h"
//#include "gps.h"
//#include "moto.h"
#include "data_transfer.h"
#include "Flash_eeprom.h"
#include "timer.h"
#include "hcsr04.h"
Control RCTarget; 
uint8_t FlyMode=0,StabilizeMode=0,GpsMode=0,AutoHighMode=0,AutoHighSensorUsed=0,HeadFreeMode=0;
float HeadFree_Yaw_Set,HeadFree_Yaw_Error;	 //��ͷģʽ����궨/ƫ��
int16_t Throttle_Standard=0;  //�������Ż�׼ֵ
float THROTTLE_Stabilize;
int16_t PID_ROLL,PID_PITCH,PID_YAW;	
	u8 i=0;
//float THROTTLE; 
float  Target_Yaw_Rate=0,Target_Roll=0,Target_Pitch=0,Target_Yaw=0,Target_Altitude=0,Altitude_Standard=0,Target_Latitude[2],Target_Longitude[2];
u8 Position_Ready = 0;

u8 Auto_Fixed_High;
float Fixed_High;
void Control_Init(void)
{
	Flash_ReadOFFSET();
	Flash_ReadPID();
	RCTarget.Throttle=0;
	RCTarget.Roll=0;
	RCTarget.Pitch=0;
	RCTarget.Yaw=0;
	pidSetIntegralLimit(&RollRate,30);
	pidSetIntegralLimit(&PitchRate,30);
	pidSetIntegralLimit(&YawRate,50);
	pidSetIntegralLimit(&Stabilize_Roll,100);
	pidSetIntegralLimit(&Stabilize_Pitch,100);
	pidSetIntegralLimit(&Stabilize_Yaw,50);
	pidSetIntegralLimit(&RollAccel,3);
	pidSetIntegralLimit(&PitchAccel,3);
	pidSetIntegralLimit(&AutoHigh_THR,15);

}
//��־��FlyMode
void Read_FlyMode(void)
{
		FlyMode = 1;	
	return ;
}

//��־��GpsMode,StabilizeMode
void Read_Gps_StabilizeMode(void)		//ƽ�����ģʽ����
{
		StabilizeMode = 1;	   	//�������Ŷ���ģʽ
		return ;
}

//��־��AutoHighMode
void Read_AutoHighMode(void)
{
		AutoHighMode = 1;		//����ģʽ
		return ;
}

//��־��HeadFreeMode
void Read_HeadFreeMode(void)
{
	HeadFreeMode=1;
		return ;
}

void Fly_Control(void)
{
	static uint32_t last_time=0, now_time=0; // �������ڼ��� ��λ us
		
	Read_FlyMode();//FlyMode=1
//	Read_Gps_StabilizeMode();//StabilizeMode=1
//	Read_AutoHighMode();
	Read_HeadFreeMode();

//	if(RCTarget.Throttle < 1020)			//������ʩ�������ŵ�ʱ���رյ�������������ơ�
//	{
//		Moto_Reflash(1000,1000,1000,1000);
//		return;
//	}

	now_time = micros();  //��ȡʱ��
	if(now_time > last_time)
	{
		PID_dt =  ((float)(now_time - last_time) / 1000000.0f);
	}
	else
	{
		last_time = now_time;
		return;
	}	
	last_time = now_time;
	
	
	switch(FlyMode)	//������ģʽ
	{ 		
		case 1:	//
				StableMode_Control();//ǰ��
				break ;

		default:
				break ;
	}			
}
void Stabilize_Mode_Conrtol(void)
{		

	//THROTTLE = RCTarget.Throttle;   

	if(HeadFreeMode == 1)  	//��ͷģʽ
	{
		HeadFree_Yaw_Error = Get_Yaw_Error(HeadFree_Yaw_Set,Mag_Yaw);
		HeadFree_Yaw_Error *= 0.0174533f; //ת���ɻ���
		Target_Roll = RCTarget.Roll * cos(HeadFree_Yaw_Error) - RCTarget.Pitch * sin(HeadFree_Yaw_Error);
		Target_Pitch = RCTarget.Roll * sin(HeadFree_Yaw_Error) + RCTarget.Pitch * cos(HeadFree_Yaw_Error);

		
	}
	else
	{
		HeadFree_Yaw_Set = Mag_Yaw;     //��ͷģʽ����궨
		Target_Roll = RCTarget.Roll;
		Target_Pitch = RCTarget.Pitch;		
	}

	if(THROTTLE<1000)	 //�����Ų�����YAW���ƣ�����PID��������
	{
		pidReset_all();  
		Target_Yaw =  Q_ANGLE.Z;
	}
	Target_Yaw =  Q_ANGLE.Z;
	Target_Yaw_Rate = 0;	
}
	
	//---------------����PWM���------------------------------------ 

void StableMode_Control(void)	 //ң��������ģʽ
{

	if(HeadFreeMode == 1)  	//��ͷģʽ
	{
		HeadFree_Yaw_Error = Get_Yaw_Error(HeadFree_Yaw_Set,Mag_Yaw);
		HeadFree_Yaw_Error *= 0.0174533f; //ת���ɻ���
		Target_Roll = RCTarget.Roll * cos(HeadFree_Yaw_Error) + RCTarget.Pitch * sin(HeadFree_Yaw_Error);
		Target_Pitch = -RCTarget.Roll * sin(HeadFree_Yaw_Error) + RCTarget.Pitch * cos(HeadFree_Yaw_Error);

	
	}
	else
	{
		HeadFree_Yaw_Set = Mag_Yaw;     //��ͷģʽ����궨
		Target_Roll = RCTarget.Roll;
		Target_Pitch = RCTarget.Pitch;		
	}
	if( Auto_Fixed_High )
		THROTTLE=RCTarget.Throttle+Auto_High_PID(Fixed_High,0);
	else
	{
		THROTTLE=RCTarget.Throttle;
		Fixed_High = HCSR04_Distance;
	}
	//}
//	if(THROTTLE<1200)	 //�����Ų�����YAW���ƣ�����PID��������
//	{
//		pidReset_all();  
//		Target_Yaw =  Q_ANGLE.Z;
//	}
	if((RCTarget.Yaw > 2 || RCTarget.Yaw < -2) )	 
	{
		Target_Yaw =  Q_ANGLE.Z;
		Target_Yaw_Rate = -RCTarget.Yaw*100;
	}
	else
	{
		Target_Yaw_Rate = 0;	
	}

		 
	
	Roll_Pitch_Yaw_AnglePID(Target_Roll,Target_Pitch,Target_Yaw,Target_Yaw_Rate); 

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;


	
	PWM_Write_Motors(); //д�����PWMͨ��
}



float Get_Current_Altitude(void)
{
	if(AutoHighSensorUsed == 0)	 //ʹ����ѹ�ƶ���
	{
		return 1234;
	}
	else
	{
//		return HCSR04_Distance;
		return 0;
	}	
}

#define Limit_Max     (int16_t)30   //ÿ��Լ���� 200 �� 
int16_t Last_PID_ROLL,Last_PID_PITCH,Last_PID_YAW;

// ��PID�����ֵ���� ƽ������������Ҫ̫������
void PWM_PID_Smooth(void)
{
   PID_ROLL = Last_PID_ROLL + Math_Constrain((PID_ROLL - Last_PID_ROLL),-Limit_Max,+Limit_Max);
   PID_PITCH = Last_PID_PITCH + Math_Constrain((PID_PITCH - Last_PID_PITCH),-Limit_Max,+Limit_Max);
   PID_YAW = Last_PID_YAW + Math_Constrain((PID_YAW - Last_PID_YAW),-Limit_Max,+Limit_Max);
   
   //����һ�ε�ֵ���棬�Ա��´μ���ʱʹ��
   Last_PID_ROLL = PID_ROLL;
   Last_PID_PITCH = PID_PITCH;
   Last_PID_YAW	= PID_YAW;
}


//�� �����������Ŀ��������� ������
#define PIDMIX(X,Y,Z) THROTTLE + PID_ROLL*X + PID_PITCH*Y +  PID_YAW*Z
int16_t motor[4] = {1000,1000,1000,1000};
void PWM_Write_Motors(void)
{

	PWM_PID_Smooth();  //ƽ����� 
	
	if(THROTTLE > 1020)	  	//������ʩ�������ŵ�ʱ�������������
	{


		motor[0] = PIDMIX(+1,+1,-1);
		motor[1] = PIDMIX(+1,-1,+1)-5; 
		motor[2] = PIDMIX(-1,-1,-1);//105��ROLL�����  40��PITCH�����
		motor[3] = PIDMIX(-1,+1,+1);
//		UserData[0] = motor[0];
//		UserData[1] = motor[1];
//		UserData[2] = motor[2];
//		UserData[3] = motor[3];	
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}
	else   	//������С�� 5%  ���������
	{
		motor[0]=1000;
		motor[1]=1000;
		motor[2]=1000;
		motor[3]=1000;
		PID_ROLL = 0;  //���еĿ���������
		PID_PITCH = 0; 
		PID_YAW = 0;
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}

}


