#include "IMU.h"
//#include "kalmanfilter.h"
#include "MPU6050.h"
#include "math.h"
#include "timer.h"
#include "data_transfer.h"
#include "control.h"
#include "hmc5883.h"

#define RtA 		57.324841f			//弧度到角度
#define AtR    	0.0174533f				//度到角度
#define Acc_G 	0.0598145f			//加速度变成cm/s2  对应+-2g 
#define Gyro_G 	0.0610351f				//角速度变成度   此参数对应陀螺2000度每秒
#define Gyro_Gr	0.0010653f				//角速度变成弧度	此参数对应陀螺2000度每秒
#define FILTER_NUM 20

S_FLOAT_XYZ Accel_Offset;			//加速度零飘
S_FLOAT_XYZ GYRO_I,ACC,ACC_PLF,GYRO_PLF;				//陀螺仪积分
S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
u8 Accel_Offset_Ok = 1;
//int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//加速度滑动窗口滤波数组
float ACC_Z=0;
float AngleOffset_Rol=0,AngleOffset_Pit=0;
float IMU_SPEED[2]={0,0},IMU_SPEED_Z=0;


/**************************实现函数********************************************
*函数原型:		void Prepare_Data(void)
*功　　能:	    每1ms采集一次数据，不停的采，第一次的时候回进行零漂滤波
*******************************************************************************/
void Prepare_Data(void)
{
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	static float start_time=0;
	float fullT;

	now_time = micros();  //读取时间
	if(now_time > last_time)
	{
		fullT =  ((float)(now_time - last_time) / 1000000.0f);
	}
	else
	{
		last_time = now_time;
		return;
	}	
	last_time = now_time;

	if(Accel_Offset_Ok)
	{
		static float tempax=0,tempay=0,tempaz=0;
		static uint16_t cnt_a=0;
		if(cnt_a==0)
		{
			Accel_Offset.X = 0;
			Accel_Offset.Y = 0;
			Accel_Offset.Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= ACC.X;
		tempay+= ACC.Y;
		tempaz+= ACC.Z;
		if(cnt_a==5000)
		{
			Accel_Offset.X=tempax/cnt_a;
			Accel_Offset.Y=tempay/cnt_a;
			Accel_Offset.Z=tempaz/cnt_a;
			cnt_a = 0;
			Accel_Offset_Ok = 1;
			//Flash_SaveOFFSET();//保存数据
			return;
		} 
		cnt_a++;		
	}
  //同样MPU6050中陀螺仪的数据也会受到震动的干扰，也要对其进行20HZ的低通滤波。
	//其中fullT是陀螺仪的采样周期，MPU6050_GYRO_LAST 是陀螺仪校准后的数据，GYRO_PLF是陀螺仪低通滤波后的数据。
	GYRO_PLF.X = GYRO_PLF.X +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.X - GYRO_PLF.X);
	GYRO_PLF.Y = GYRO_PLF.Y +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.Y - GYRO_PLF.Y);
	GYRO_PLF.Z = GYRO_PLF.Z +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.Z - GYRO_PLF.Z);
	
	//陀螺仪积分
	GYRO_I.Z += MPU6050_GYRO_LAST.Z*fullT*Gyro_G;
	/*
	if(GYRO_I.Z >180.0f) GYRO_I.Z = GYRO_I.Z-360.0f;	//转[-180.0,+180.0]
	else if(GYRO_I.Z <-180.0f) GYRO_I.Z = 360.0f + GYRO_I.Z;
	*/
	
  // MPU6050中加速度计输出的数据在飞行器飞行时由于电机的震动，会带有很大的震动噪声
	//需要对原始数据进行20HZ的低通滤波，以尽可能的过滤掉噪声的干扰。如下
	//其中fullT是加速度计的采样周期，MPU6050_ACC_LAST 是加速度计校准后的，ACC_PLF是加速度计低通滤波后的数据。
	ACC_PLF.X = ACC_PLF.X +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.X - ACC_PLF.X);
	ACC_PLF.Y = ACC_PLF.Y +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.Y - ACC_PLF.Y);
	ACC_PLF.Z = ACC_PLF.Z +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.Z - ACC_PLF.Z);

	//将滤波后的加速度计转换到水平坐标系并剔除重力加速度，Accel_Offset.Z;为重力加速度
	//但是三个量实际是速度
	ACC.X = (ACC_PLF.X*cos(-Pitch)+ACC_PLF.Z*sin(-Pitch))*Acc_G;
	ACC.Y = (ACC_PLF.Y*cos(-Roll)+ACC_PLF.X*sin(-Roll)*sin(-Pitch)-ACC_PLF.Z*sin(-Roll)*cos(-Pitch))*Acc_G;
	ACC.Z = ((-ACC_PLF.X*cos(-Roll)*sin(-Pitch)+ACC_PLF.Y*sin(-Roll)+ACC_PLF.Z*cos(-Roll)*cos(-Pitch)))*Acc_G - Accel_Offset.Z;
	if(start_time<5)	  //等数据稳定下来再积分
	{
		start_time+= fullT;
	}	
	else
	{
		//互补滤波计算出飞行器XYZ速度
		if(1)
		{
		}
	}

	/*	
		Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
		R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
	*/
}

void Get_Attitude(void)
{

//	IMUupdate(GYRO_PLF.X*Gyro_Gr,GYRO_PLF.Y*Gyro_Gr,GYRO_PLF.Z*Gyro_Gr,ACC_PLF.X,ACC_PLF.Y,ACC_PLF.Z);	//*0.0174转成弧度

	AHRSupdate(GYRO_PLF.X*Gyro_Gr,
				GYRO_PLF.Y*Gyro_Gr,
				GYRO_PLF.Z*Gyro_Gr,
				ACC_PLF.X,ACC_PLF.Y,ACC_PLF.Z,
				AVG_MAG.X,AVG_MAG.Y,AVG_MAG.Z);	//*0.0174转成弧度
}
////////////////////////////////////////////////////////////////////////////////
//#define Kp 4.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.0017f                 // half the sample period采样周期的一半

float Roll,Pitch,Yaw,Mag_Yaw,HalfT;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	float Kp = 1.0f;                        // proportional gain governs rate of convergence to accelerometer/magnetometer
	float Ki = 0.001f;                          // integral gain governs rate of convergence of gyroscope biases
	float norm,Accel_magnitude;
	//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;
	
	// 先把这些用得到的值算好
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	//  float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	//  float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
		return;

  //---------计算加速度信任度----------------
	Accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
	Accel_magnitude = Accel_magnitude / 16384.0f; // Scale to gravity.
	if(Accel_magnitude < 1.0f)
	{
		Accel_magnitude = 1.0f - Accel_magnitude;
	}
	else
	{ 
	Accel_magnitude = Accel_magnitude - 1.0f;
	}
	if(Accel_magnitude <0.3f)
	{
		Accel_magnitude = (0.3f - Accel_magnitude) * 3.3333f;
  	}
	else
	{
		Accel_magnitude = (Accel_magnitude - 0.3f) * 3.3333f;	
	}
////
	now_time = micros();  //读取时间这里用到定时器7 
	if(now_time > last_time)
	{
		HalfT =  ((float)(now_time - last_time) / 2000000.0f);
	}
	else
	{
		last_time = now_time;
		return;
	}	
	last_time = now_time;
		//////////////////////	
	norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;
	
	// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
	vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) * Accel_magnitude;                           					 //向量外积在相减得到差分就是误差
	ey = (az*vx - ax*vz) * Accel_magnitude;
	ez = (ax*vy - ay*vx) * Accel_magnitude;
	
	exInt = exInt + ex * Ki;								  //对误差进行积分
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;					   							//将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   							//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
	
	// integrate quaternion rate and normalise						   //四元素的微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*HalfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*HalfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*HalfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*HalfT;
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	Roll = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);
	Pitch = asin(-2 * q1 * q3 + 2 * q0* q2);

//	Q_ANGLE.Z = GYRO_I.Z;//atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
//	Q_ANGLE.Z = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
	Q_ANGLE.Y = Pitch * 57.29578f - AngleOffset_Pit;//俯仰角
	Q_ANGLE.X = Roll * 57.29578f - AngleOffset_Rol;//横滚角
//	printf("tmp : %8.3f %8.3f %8.3f\r\n",Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z);
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
	static float start_time=0;
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	float Kp = 10.0f;                        // proportional gain governs rate of convergence to accelerometer/magnetometer
	float Ki = 0.000f;                          // integral gain governs rate of convergence of gyroscope biases
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, Accel_magnitude;
	float halfT;	
	// 先把这些用得到的值算好
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

  //---------计算加速度信任度----------------
	Accel_magnitude = sqrt(ax*ax + ay*ay + az*az);
	Accel_magnitude = Accel_magnitude / 16384.0f; // Scale to gravity.
	if(Accel_magnitude < 1.0f)
	{
		Accel_magnitude = 1.0f - Accel_magnitude;
	}
	else
	{ 
	Accel_magnitude = Accel_magnitude - 1.0f;
	}
	if(Accel_magnitude <0.3f)
	{
		Accel_magnitude = (0.3f - Accel_magnitude) * 3.3333f;
  	}
	else
	{
		Accel_magnitude = (Accel_magnitude - 0.3f) * 3.3333f;	
	}

	if(ax*ay*az==0 || mx*my*mz==0)
		return;

	now_time = micros();  //读取时间
	if(now_time > last_time)
	{
		halfT =  ((float)(now_time - last_time) / 2000000.0f);
	}
	else
	{			 
		last_time = now_time;
		return;
	}	
	last_time = now_time;
	
	if(start_time<5)	  //秒内让姿态快速稳定下来
	{
		start_time+= 2.0f*halfT;
		//GYRO_I.Z = Mag_Yaw;
		HeadFree_Yaw_Set = Mag_Yaw;	 //无头模式航向标定
	}
	else
	{
		Kp = 1.0f;
		Ki = 0.0f;	
	}	
	norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;
	norm = sqrt(mx*mx + my*my + mz*mz);
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;

   // compute reference direction of flux
    hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
    hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
    hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

	// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
	vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2); 
		
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy)* Accel_magnitude + (my*wz - mz*wy);                          					 //向量外积在相减得到差分就是误差
	ey = (az*vx - ax*vz)* Accel_magnitude + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx)* Accel_magnitude + (mx*wy - my*wx);
	
	exInt = exInt + ex * Ki;								  //对误差进行积分
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;					   							//将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   							//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
	
	// integrate quaternion rate and normalise						   //四元素的微分方程
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	
	Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);
	Pitch = -asin(-2 * q1 * q3 + 2 * q0* q2);
	Mag_Yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.29578;
//	Q_ANGLE.Z = -Mag_Yaw; // yaw
	Q_ANGLE.Z  = atan2(my*cos(-Roll) + mx*sin(-Roll)*sin(-Pitch) - mz*sin(-Roll)*cos(-Pitch), mx*cos(-Pitch)+mz*sin(-Pitch))*180/3.14159265;
	Q_ANGLE.Y = Pitch * 180 / 3.1415926f - AngleOffset_Pit;
	Q_ANGLE.X = Roll * 180 / 3.1415926f - AngleOffset_Rol;
}
