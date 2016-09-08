#include "IMU.h"
//#include "kalmanfilter.h"
#include "MPU6050.h"
#include "math.h"
#include "timer.h"
#include "data_transfer.h"
#include "control.h"
#include "hmc5883.h"

#define RtA 		57.324841f			//���ȵ��Ƕ�
#define AtR    	0.0174533f				//�ȵ��Ƕ�
#define Acc_G 	0.0598145f			//���ٶȱ��cm/s2  ��Ӧ+-2g 
#define Gyro_G 	0.0610351f				//���ٶȱ�ɶ�   �˲�����Ӧ����2000��ÿ��
#define Gyro_Gr	0.0010653f				//���ٶȱ�ɻ���	�˲�����Ӧ����2000��ÿ��
#define FILTER_NUM 20

S_FLOAT_XYZ Accel_Offset;			//���ٶ���Ʈ
S_FLOAT_XYZ GYRO_I,ACC,ACC_PLF,GYRO_PLF;				//�����ǻ���
S_FLOAT_XYZ Q_ANGLE;			//��Ԫ��������ĽǶ�
u8 Accel_Offset_Ok = 1;
//int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//���ٶȻ��������˲�����
float ACC_Z=0;
float AngleOffset_Rol=0,AngleOffset_Pit=0;
float IMU_SPEED[2]={0,0},IMU_SPEED_Z=0;


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Prepare_Data(void)
*��������:	    ÿ1ms�ɼ�һ�����ݣ���ͣ�Ĳɣ���һ�ε�ʱ��ؽ�����Ư�˲�
*******************************************************************************/
void Prepare_Data(void)
{
	static uint32_t last_time=0, now_time=0; // �������ڼ��� ��λ us
	static float start_time=0;
	float fullT;

	now_time = micros();  //��ȡʱ��
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
			//Flash_SaveOFFSET();//��������
			return;
		} 
		cnt_a++;		
	}
  //ͬ��MPU6050�������ǵ�����Ҳ���ܵ��𶯵ĸ��ţ�ҲҪ�������20HZ�ĵ�ͨ�˲���
	//����fullT�������ǵĲ������ڣ�MPU6050_GYRO_LAST ��������У׼������ݣ�GYRO_PLF�������ǵ�ͨ�˲�������ݡ�
	GYRO_PLF.X = GYRO_PLF.X +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.X - GYRO_PLF.X);
	GYRO_PLF.Y = GYRO_PLF.Y +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.Y - GYRO_PLF.Y);
	GYRO_PLF.Z = GYRO_PLF.Z +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_GYRO_LAST.Z - GYRO_PLF.Z);
	
	//�����ǻ���
	GYRO_I.Z += MPU6050_GYRO_LAST.Z*fullT*Gyro_G;
	/*
	if(GYRO_I.Z >180.0f) GYRO_I.Z = GYRO_I.Z-360.0f;	//ת[-180.0,+180.0]
	else if(GYRO_I.Z <-180.0f) GYRO_I.Z = 360.0f + GYRO_I.Z;
	*/
	
  // MPU6050�м��ٶȼ�����������ڷ���������ʱ���ڵ�����𶯣�����кܴ��������
	//��Ҫ��ԭʼ���ݽ���20HZ�ĵ�ͨ�˲����Ծ����ܵĹ��˵������ĸ��š�����
	//����fullT�Ǽ��ٶȼƵĲ������ڣ�MPU6050_ACC_LAST �Ǽ��ٶȼ�У׼��ģ�ACC_PLF�Ǽ��ٶȼƵ�ͨ�˲�������ݡ�
	ACC_PLF.X = ACC_PLF.X +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.X - ACC_PLF.X);
	ACC_PLF.Y = ACC_PLF.Y +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.Y - ACC_PLF.Y);
	ACC_PLF.Z = ACC_PLF.Z +  (fullT / ( 7.9577e-3f + fullT)) * ((float)MPU6050_ACC_LAST.Z - ACC_PLF.Z);

	//���˲���ļ��ٶȼ�ת����ˮƽ����ϵ���޳��������ٶȣ�Accel_Offset.Z;Ϊ�������ٶ�
	//����������ʵ�����ٶ�
	ACC.X = (ACC_PLF.X*cos(-Pitch)+ACC_PLF.Z*sin(-Pitch))*Acc_G;
	ACC.Y = (ACC_PLF.Y*cos(-Roll)+ACC_PLF.X*sin(-Roll)*sin(-Pitch)-ACC_PLF.Z*sin(-Roll)*cos(-Pitch))*Acc_G;
	ACC.Z = ((-ACC_PLF.X*cos(-Roll)*sin(-Pitch)+ACC_PLF.Y*sin(-Roll)+ACC_PLF.Z*cos(-Roll)*cos(-Pitch)))*Acc_G - Accel_Offset.Z;
	if(start_time<5)	  //�������ȶ������ٻ���
	{
		start_time+= fullT;
	}	
	else
	{
		//�����˲������������XYZ�ٶ�
		if(1)
		{
		}
	}

	/*	
		Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
		R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
	*/
}

void Get_Attitude(void)
{

//	IMUupdate(GYRO_PLF.X*Gyro_Gr,GYRO_PLF.Y*Gyro_Gr,GYRO_PLF.Z*Gyro_Gr,ACC_PLF.X,ACC_PLF.Y,ACC_PLF.Z);	//*0.0174ת�ɻ���

	AHRSupdate(GYRO_PLF.X*Gyro_Gr,
				GYRO_PLF.Y*Gyro_Gr,
				GYRO_PLF.Z*Gyro_Gr,
				ACC_PLF.X,ACC_PLF.Y,ACC_PLF.Z,
				AVG_MAG.X,AVG_MAG.Y,AVG_MAG.Z);	//*0.0174ת�ɻ���
}
////////////////////////////////////////////////////////////////////////////////
//#define Kp 4.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.0017f                 // half the sample period�������ڵ�һ��

float Roll,Pitch,Yaw,Mag_Yaw,HalfT;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
	static uint32_t last_time=0, now_time=0; // �������ڼ��� ��λ us
	float Kp = 1.0f;                        // proportional gain governs rate of convergence to accelerometer/magnetometer
	float Ki = 0.001f;                          // integral gain governs rate of convergence of gyroscope biases
	float norm,Accel_magnitude;
	//  float hx, hy, hz, bx, bz;
	float vx, vy, vz;// wx, wy, wz;
	float ex, ey, ez;
	
	// �Ȱ���Щ�õõ���ֵ���
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

  //---------������ٶ����ζ�----------------
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
	now_time = micros();  //��ȡʱ�������õ���ʱ��7 
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
	norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
	ax = ax /norm;
	ay = ay / norm;
	az = az / norm;
	
	// estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
	vx = 2*(q1q3 - q0q2);												//��Ԫ����xyz�ı�ʾ
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
	
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) * Accel_magnitude;                           					 //�������������õ���־������
	ey = (az*vx - ax*vz) * Accel_magnitude;
	ez = (ax*vy - ay*vx) * Accel_magnitude;
	
	exInt = exInt + ex * Ki;								  //�������л���
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;					   							//�����PI�󲹳��������ǣ����������Ư��
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   							//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
	
	// integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
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
	Q_ANGLE.Y = Pitch * 57.29578f - AngleOffset_Pit;//������
	Q_ANGLE.X = Roll * 57.29578f - AngleOffset_Rol;//�����
//	printf("tmp : %8.3f %8.3f %8.3f\r\n",Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z);
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
	static float start_time=0;
	static uint32_t last_time=0, now_time=0; // �������ڼ��� ��λ us
	float Kp = 10.0f;                        // proportional gain governs rate of convergence to accelerometer/magnetometer
	float Ki = 0.000f;                          // integral gain governs rate of convergence of gyroscope biases
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, Accel_magnitude;
	float halfT;	
	// �Ȱ���Щ�õõ���ֵ���
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

  //---------������ٶ����ζ�----------------
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

	now_time = micros();  //��ȡʱ��
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
	
	if(start_time<5)	  //��������̬�����ȶ�����
	{
		start_time+= 2.0f*halfT;
		//GYRO_I.Z = Mag_Yaw;
		HeadFree_Yaw_Set = Mag_Yaw;	 //��ͷģʽ����궨
	}
	else
	{
		Kp = 1.0f;
		Ki = 0.0f;	
	}	
	norm = sqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
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

	// estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
	vx = 2*(q1q3 - q0q2);												//��Ԫ����xyz�ı�ʾ
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;
  wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2); 
		
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy)* Accel_magnitude + (my*wz - mz*wy);                          					 //�������������õ���־������
	ey = (az*vx - ax*vz)* Accel_magnitude + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx)* Accel_magnitude + (mx*wy - my*wx);
	
	exInt = exInt + ex * Ki;								  //�������л���
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;					   							//�����PI�󲹳��������ǣ����������Ư��
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;				   							//�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�
	
	// integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
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
