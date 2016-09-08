#ifndef PID_H_
#define PID_H_
#include "stm32f4xx.h"

#define DEFAULT_PID_INTEGRATION_LIMIT  1000.0
extern float THROTTLE;
struct Quad_PID{
	float target;  // 目标值
	float current; // 当前值
	float merror;
	float last_error;
	float Integrator;	//当前积分值
	float deriv;
	float iLimit;
	float Kp;	   //比例 
	float Ki;	   //积分
	float Kd;	   //微分
	float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
	float PID_out;   //当前PID 的输出
};

extern struct Quad_PID 
				RollRate,		  // 滚转 角速率PID
				PitchRate,		  // 府仰 角速率PID
				YawRate,          //航向 角速率PID
				Stabilize_Roll,  // 滚转 PID
				Stabilize_Pitch,  // 府仰 PID
				Stabilize_Yaw,	  //航向 PID
				Climb,            //爬升 PID
				AutoHigh_THR,	  //定高 PID
				RollAccel,		  //滚转 加速度PID
				PitchAccel,      //滚转 角速度PID
				Position_Hold,	  //位置 定点PID
				Position_Speed	  //速度 PID
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
