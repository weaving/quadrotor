#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx.h"
typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;
typedef struct Control{vs16 Throttle;float Roll,Pitch,Yaw;}Control;

//extern PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS,PID_PID_1,PID_PID_2;
extern Control RCTarget ;
extern float HeadFree_Yaw_Set;
extern u8 FlyMode,StabilizeMode,AutoHighMode,AutoHighSensorUsed,HeadFreeMode;

void Fly_Control(void);
void Manual_Mode_Control(void);
void Stabilize_Mode_Conrtol(void);
void GPS_Mode_Control(void);
void Stabilize_Test_Mode_Conrtol(void);
void AutoHigh_Select_Sensor(void);
float Get_Current_Altitude(void);
void PWM_PID_Smooth(void);
void PWM_Write_Motors(void);
void PWM_Write1_Motors(void);
void PWM_Damp_OUT(float Target_Height);//前进
void PWM_Damp_OUT1(float Target_Height);//前进
void PWM_sethigh_OUT(float Target_Height);//后退
void zuoguai(float Target_Height);
void youguai(float Target_Height);
void PWM_Drop_OUT(void);//降落
//void CONTROL(float rol_now, float pit_now, float yaw_now, float rol_tar, float pit_tar, float yaw_tar);
void Control_Init(void);
#endif
