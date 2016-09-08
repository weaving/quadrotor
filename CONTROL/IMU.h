#ifndef __IMU_H
#define __IMU_H 	

#include "stm32f4xx.h"
#include "delay.h"
#include "MPU6050.h"
#define M_PI  (float)3.1415926535
	
void IMU_init(void); //初始化
void IMU_getYawPitchRoll(float * ypr); //更新姿态

typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
extern S_FLOAT_XYZ GYRO_I;
extern S_FLOAT_XYZ ACC,ACC_PLF,Accel_Offset;		
extern float Roll,Pitch,Yaw,Mag_Yaw;
extern float AngleOffset_Rol,AngleOffset_Pit;
extern float ACC_Z;
extern float IMU_SPEED[2],IMU_SPEED_Z;
extern u8 Accel_Offset_Ok;
		

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
#endif
