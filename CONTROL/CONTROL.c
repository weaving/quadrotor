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
float HeadFree_Yaw_Set,HeadFree_Yaw_Error;	 //无头模式航向标定/偏差
int16_t Throttle_Standard=0;  //定高油门基准值
float THROTTLE_Stabilize;
int16_t PID_ROLL,PID_PITCH,PID_YAW;	
	u8 i=0;
//float THROTTLE; 
float  Target_Yaw_Rate=0,Target_Roll=0,Target_Pitch=0,Target_Yaw=0,Target_Altitude=0,Altitude_Standard=0,Target_Latitude[2],Target_Longitude[2];
u8 Position_Ready = 0;

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
//标志：FlyMode
void Read_FlyMode(void)
{
		FlyMode = 1;	
	return ;
}

//标志：GpsMode,StabilizeMode
void Read_Gps_StabilizeMode(void)		//平衡飞行模式设置
{
		StabilizeMode = 1;	   	//智能油门定高模式
		return ;
}

//标志：AutoHighMode
void Read_AutoHighMode(void)
{
		AutoHighMode = 1;		//定高模式
		return ;
}

//标志：HeadFreeMode
void Read_HeadFreeMode(void)
{
	HeadFreeMode=0;
		return ;
}

void Fly_Control(void)
{
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
		
	Read_FlyMode();//FlyMode=1
//	Read_Gps_StabilizeMode();//StabilizeMode=1
//	Read_AutoHighMode();
	Read_HeadFreeMode();

//	if(RCTarget.Throttle < 1020)			//保护措施，当油门低时，关闭电机，不启动控制。
//	{
//		Moto_Reflash(1000,1000,1000,1000);
//		return;
//	}

	now_time = micros();  //读取时间
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
	
//	HeadFree_Yaw_Set=53;
//	HeadFree_Yaw_Error = Get_Yaw_Error(HeadFree_Yaw_Set,Mag_Yaw);
//	HeadFree_Yaw_Error *= 0.0174533f; //转换成弧度
//	Target_Roll = RCTarget.Roll * cos(HeadFree_Yaw_Error) - RCTarget.Pitch * sin(HeadFree_Yaw_Error);
//	Target_Pitch = RCTarget.Roll * sin(HeadFree_Yaw_Error) + RCTarget.Pitch * cos(HeadFree_Yaw_Error);
	
	switch(FlyMode)	//检测飞行模式
	{ 		
		case 1:	//
				PWM_Damp_OUT(70.0);//前进
				break ;

		default:
				break ;
	}			
}
void Stabilize_Mode_Conrtol(void)
{		

	//THROTTLE = RCTarget.Throttle;   

	if(HeadFreeMode == 1)  	//无头模式
	{
		HeadFree_Yaw_Error = Get_Yaw_Error(HeadFree_Yaw_Set,Mag_Yaw);
		HeadFree_Yaw_Error *= 0.0174533f; //转换成弧度
		Target_Roll = RCTarget.Roll * cos(HeadFree_Yaw_Error) - RCTarget.Pitch * sin(HeadFree_Yaw_Error);
		Target_Pitch = RCTarget.Roll * sin(HeadFree_Yaw_Error) + RCTarget.Pitch * cos(HeadFree_Yaw_Error);
	}
	else
	{
		HeadFree_Yaw_Set = Mag_Yaw;     //无头模式航向标定
		Target_Roll = RCTarget.Roll;
		Target_Pitch = RCTarget.Pitch;		
	}

	if(THROTTLE<1000)	 //低油门不启用YAW控制，所有PID积分清零
	{
		pidReset_all();  
		Target_Yaw =  Q_ANGLE.Z;
	}
	Target_Yaw =  Q_ANGLE.Z;
	Target_Yaw_Rate = 0;	
}
	
	//---------------计算PWM输出------------------------------------ 

void PWM_Damp_OUT(float Target_Height)	 //前进
{
//	THROTTLE=1300;
// 	Stabilize_Mode_Conrtol();
// 	if(i==0)
// 	{
// 			if(HCSR04_Distance<Target_Height)
// 			{
// 								THROTTLE=THROTTLE+0.002*(Target_Height-HCSR04_Distance);
// 								if(THROTTLE>1520)
// 											THROTTLE=1520;//12.0 1560//11.9/11.8 1570
// 			}
// 			if(HCSR04_Distance>=Target_Height)
// 			{
// 				i=1;
// 				THROTTLE_Stabilize=THROTTLE;
// 			}
// 	}
	//else
	//{
	THROTTLE=RCTarget.Throttle+Auto_High_PID(HCSR04_Distance_Last,0);
	//}
//	Roll_Pitch_Yaw_AnglePID(RCTarget.Roll,RCTarget.Pitch,0,Target_Yaw_Rate); //智能油门定高模式
//	Roll_Pitch_Yaw_AnglePID(RCTarget.Roll,0,0,Target_Yaw_Rate);
	Target_Yaw =  Q_ANGLE.Z;
	Target_Yaw_Rate = -RCTarget.Yaw*100;
	Target_Pitch = RCTarget.Pitch;
	Target_Roll = RCTarget.Roll;
	
	Roll_Pitch_Yaw_AnglePID(Target_Roll,Target_Pitch,Target_Yaw,Target_Yaw_Rate); 

	PID_PITCH = PitchRate.PID_out;
	PID_ROLL = RollRate.PID_out;
	PID_YAW = YawRate.PID_out;
	PWM_Write1_Motors(); //写输出到PWM通道
}



float Get_Current_Altitude(void)
{
	if(AutoHighSensorUsed == 0)	 //使用气压计定高
	{
		return 1234;
	}
	else
	{
//		return HCSR04_Distance;
		return 0;
	}	
}

#define Limit_Max     (int16_t)30   //每秒约计算 200 次 
int16_t Last_PID_ROLL,Last_PID_PITCH,Last_PID_YAW;

// 将PID计算的值进行 平滑处理。动作不要太生猛了
void PWM_PID_Smooth(void)
{
   PID_ROLL = Last_PID_ROLL + Math_Constrain((PID_ROLL - Last_PID_ROLL),-Limit_Max,+Limit_Max);
   PID_PITCH = Last_PID_PITCH + Math_Constrain((PID_PITCH - Last_PID_PITCH),-Limit_Max,+Limit_Max);
   PID_YAW = Last_PID_YAW + Math_Constrain((PID_YAW - Last_PID_YAW),-Limit_Max,+Limit_Max);
   
   //将这一次的值保存，以便下次计算时使用
   Last_PID_ROLL = PID_ROLL;
   Last_PID_PITCH = PID_PITCH;
   Last_PID_YAW	= PID_YAW;
}


//将 四轴计算出来的控制量进行 混合输出
#define PIDMIX(X,Y,Z) THROTTLE + PID_ROLL*X + PID_PITCH*Y +  PID_YAW*Z
int16_t motor[4] = {1000,1000,1000,1000};
void PWM_Write_Motors(void)
{

	PWM_PID_Smooth();  //平滑输出 
	
	if(THROTTLE > 1020)	  	//保护措施，当油门低时，不启动电机。
	{
		motor[0] = PIDMIX(-1,-1,+1);
		motor[1] = PIDMIX(+1,-1,-1); 
		motor[2] = PIDMIX(+1,+1,+1)+10;//105是ROLL方向的  40是PITCH方向的
		motor[3] = PIDMIX(-1,+1,-1)+10;
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}
	else   	//油门量小于 5%  不启动电机
	{
		motor[0]=1000;
		motor[1]=1000;
		motor[2]=1000;
		motor[3]=1000;
		PID_ROLL = 0;  //所有的控制量清零
		PID_PITCH = 0; 
		PID_YAW = 0;
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}
  }
void PWM_Write1_Motors(void)
{

	PWM_PID_Smooth();  //平滑输出 
	
	if(THROTTLE > 1020)	  	//保护措施，当油门低时，不启动电机。
	{

		
		motor[0] = PIDMIX(+1,+1,+1);
		motor[1] = PIDMIX(+1,-1,-1)-5; 
		
		motor[2] = PIDMIX(-1,-1,+1);//105是ROLL方向的  40是PITCH方向的
			

		motor[3] = PIDMIX(-1,+1,-1);


//	
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}
	else   	//油门量小于 5%  不启动电机
	{
		motor[0]=1000;
		motor[1]=1000;
		motor[2]=1000;
		motor[3]=1000;
		PID_ROLL = 0;  //所有的控制量清零
		PID_PITCH = 0; 
		PID_YAW = 0;
		Moto_Reflash(motor[0],motor[1],motor[2],motor[3]);
	}



}


