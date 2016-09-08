#ifndef PID_H_
#define PID_H_
#include "stm32f4xx.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  1000.0
extern float THROTTLE;
struct Quad_PID{
	float target;  // Ŀ��ֵ
	float current; // ��ǰֵ
	float merror;
	float last_error;
	float Integrator;	//��ǰ����ֵ
	float deriv;
	float iLimit;
	float Kp;	   //���� 
	float Ki;	   //����
	float Kd;	   //΢��
	float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
	float PID_out;   //��ǰPID �����
};

extern struct Quad_PID 
				RollRate,		  // ��ת ������PID
				PitchRate,		  // ���� ������PID
				YawRate,          //���� ������PID
				Stabilize_Roll,  // ��ת PID
				Stabilize_Pitch,  // ���� PID
				Stabilize_Yaw,	  //���� PID
				Climb,            //���� PID
				AutoHigh_THR,	  //���� PID
				RollAccel,		  //��ת ���ٶ�PID
				PitchAccel,      //��ת ���ٶ�PID
				Position_Hold,	  //λ�� ����PID
				Position_Speed	  //�ٶ� PID
				;
extern float PID_dt,Climb_last_out;
extern float GPS_ROLL,GPS_PITCH;

void pidInit(struct Quad_PID* pid, const float kp,
             const float ki, const float kd);
float pidUpdate(struct Quad_PID* pid, float measured,float dt);
float pidUpdate_Yaw(struct Quad_PID* pid, float dt);
void pidSetIntegralLimit(struct Quad_PID* pid, float limit);
void pidSetError(struct Quad_PID* pid, float err);
void pidReset(struct Quad_PID* pid);
void pidReset_all(void);
void pidSetTarget(struct Quad_PID* pid, float target);
void pidSetKp(struct Quad_PID* pid, float kp);
void pidSetKi(struct Quad_PID* pid, float ki);
void pidSetKd(struct Quad_PID* pid, float kd);
void pidSetMeasured(struct Quad_PID* pid, float measured);
void Roll_Pitch_Yaw_RatePID(float Rate_roll,float Rate_pitch,float Rate_yaw);
void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw,float Rate_yaw);
int16_t Auto_High_PID(float TargetHigh,uint8_t isRate);
void Roll_Pitch_AccelPID(float AccelRoll,float AccelPitch,float Angle_yaw,float Rate_yaw);
int16_t Math_Constrain(int16_t value,int16_t min,int16_t max);
float Math_fConstrain(float value, float min, float max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1,int16_t value2);
int16_t Math_max(int16_t value1,int16_t value2);
float Get_Yaw_Error(float set,float currt);
void GPS_Position_Hold(float * Target_Lat,float * Target_Lon);
void Position_Hold_Reset(void);
void RemainHover(float currentHeight, float targetHeight);
#endif /* PID_H_ */
