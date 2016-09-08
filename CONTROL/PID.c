#include "PID.h"
#include "CONTROL.h"
#include "IMU.h"
#include "timer.h"
#include "mpu6050.h"
#include "hcsr04.h"
//#include "gps.h"
#include "math.h"
#include "delay.h"
#include "data_transfer.h"
#include "sys.h"
#include "includes.h"

float Climb_test;
u16 old_THROTTLE; //¡î Ìí¼Ó¡î ¡î ¡î ¡î ¡î¶¨¸ß¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß  ¡î ¡î ¡î ¡î ¡î ¡î ¡î
float THROTTLE;//¡î Ìí¼Ó¡î ¡î ¡î ¡î ¡î¶¨¸ß¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß ¶¨¸ß  ¡î ¡î ¡î ¡î ¡î ¡î ¡î
float PID_dt ;
float ALT_Update_Interval ;
float GPS_PITCH=0,GPS_ROLL=0;
struct Quad_PID 
				RollRate,		  // ¹ö×ª ½ÇËÙÂÊPID
				PitchRate,		  // ¸®Ñö ½ÇËÙÂÊPID
				YawRate,          //º½Ïò ½ÇËÙÂÊPID
				Stabilize_Roll,  // ¹ö×ª ½Ç¶ÈPID
				Stabilize_Pitch,  // ¸®Ñö ½Ç¶ÈPID
				Stabilize_Yaw,	  //º½Ïò ½Ç¶ÈPID
				Climb,            //ÅÀÉý PID
				AutoHigh_THR,	  //¶¨¸ß PID
				RollAccel,		  //¹ö×ª ¼ÓËÙ¶ÈPID
				PitchAccel,      //¹ö×ª ½ÇËÙ¶ÈPID
				Position_Hold,	  //GPS ¶¨µãPID
				Position_Speed	  //ËÙ¶È PID
				;
uint16_t throttle=0;
extern float old_HCSR04_Distance,V;
void pidInit(struct Quad_PID* pid, float kp,
             float ki, float kd)
{
  pid->merror = 0;
  pid->last_error = 0;
  pid->Integrator = 0;
  pid->deriv = 0;
  pid->target = 0;
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;
  
  pid->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
}
//±ÈÀý*Æ«²î+»ý·Ö*Æ«²î»ýÀÛ+Î¢·Ö*Æ«²î±ä»¯¡£
//Æ«²î=¸ø¶¨ÖµÓëÊä³öÖµµÄ²é
float pidUpdate(struct Quad_PID* pid, float measured,float dt)//¶ÔÄ£ÄâPIDµÄËã·¨±íÊ¾,dt±íÊ¾µÄÓ¦¸ÃÊÇÊ±¼ä²î
{              //´ýÐÞÕýÁ¿£¬Êµ¼ÊÖµ£¬Ê±¼ä²î
  float output,temp;
  pid->current = measured;

  pid->merror = pid->target - measured;
  pid->Integrator += pid->Ki * pid->merror;
  if (pid->Integrator > pid->iLimit)//×ö»ý·ÖÏÞÖÆ
  {
    pid->Integrator = pid->iLimit;
  }
  else if (pid->Integrator < -pid->iLimit)
  {
    pid->Integrator = -pid->iLimit;
  }

  temp = (pid->merror - pid->last_error);//Õâ¸öÊÇÎ¢·ÖÁ¿=ÕâÒ»´Î-ÉÏÒ»´Î¡
	
	
  pid->deriv = pid->deriv + (temp - pid->deriv) * (dt / ( 7.9577e-3f + dt));//Õâ¸ö¿ÉÒÔ²»Òª°Ñ£¿
  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;
  pid->PID_out = output = 	pid->outP +
           					pid->outI +
           					pid->outD;

  pid->last_error = pid->merror;

  return output;
}
float pidUpdate_Yaw(struct Quad_PID* pid, float dt)
{
  float output,temp;

  pid->Integrator += pid->Ki * pid->merror;
  if (pid->Integrator > pid->iLimit)
  {
    pid->Integrator = pid->iLimit;
  }
  else if (pid->Integrator < -pid->iLimit)
  {
    pid->Integrator = -pid->iLimit;
  }

  temp = (pid->merror - pid->last_error);
  pid->deriv = pid->deriv + (temp - pid->deriv) * (dt / ( 7.9577e-3f + dt));

  pid->outP = pid->Kp * pid->merror;
  pid->outI = pid->Integrator;
  pid->outD = pid->Kd * pid->deriv;

  pid->PID_out = output = 	pid->outP +
           					pid->outI +
           					pid->outD;

  pid->last_error = pid->merror;

  return output;
}

void pidSetError(struct Quad_PID* pid, float err)//ÉèÖÃÆ«²îÖµ
{
  pid->merror = err;
}

void pidSetIntegralLimit(struct Quad_PID* pid, float limit)//ÉèÖÃ»ý·ÖÉÏÏÞ
{
  pid->iLimit = limit;
}

void pidReset(struct Quad_PID* pid)//¸´Î»Æ«²î»ýÀÛÖµ£¬Ò²¾ÍÊÇÇåÁã
{
  //pid->merror = 0;
  //pid->last_error = 0;
  pid->Integrator = 0;
  //pid->deriv = 0;
}

void pidReset_all(void)//¸´Î»ËùÓÐµÄÁ¿£¬ÓÐ¹ØÆ«²î»ýÀÛÕâ²¿·ÖµÄÖµ£¬Ò²¾ÍÊÇÇåÁã¡£
{
	pidReset(&RollRate);		  // ¹ö×ª ½ÇËÙÂÊPID
	pidReset(&PitchRate);		  // ¸®Ñö ½ÇËÙÂÊPID
	pidReset(&YawRate);          //º½Ïò ½ÇËÙÂÊPID
	pidReset(&Stabilize_Roll);  // ¹ö×ª ½Ç¶ÈPID
	pidReset(&Stabilize_Pitch);  // ¸®Ñö ½Ç¶ÈPID
	pidReset(&Stabilize_Yaw);	  //º½Ïò ½Ç¶ÈPID
	pidReset(&Climb);            //ÅÀÉý PID
	pidReset(&AutoHigh_THR);	  //¶¨¸ß PID
	pidReset(&RollAccel);		  //¹ö×ª ¼ÓËÙ¶ÈPID
	pidReset(&PitchAccel);      //¹ö×ª ½ÇËÙ¶ÈPID
	pidReset(&Position_Hold);	  //GPS ¶¨µãPID
	pidReset(&Position_Speed);	  //ËÙ¶È PID
	Climb_last_out = 0;	
}

void pidSetTarget(struct Quad_PID* pid, float target) ///ÉèÖÃÄ¿±êÖµ
{
  pid->target = target;
}


void pidSetKp(struct Quad_PID* pid, float kp)//ÉèÖÃp²ÎÊý
{
  pid->Kp = kp;
}

void pidSetKi(struct Quad_PID* pid, float ki)//ÉèÖÃi²ÎÊý
{
  pid->Ki = ki;
}

void pidSetKd(struct Quad_PID* pid, float kd)//ÉèÖÃd²ÎÊý
{
  pid->Kd = kd;
}

void pidSetMeasured(struct Quad_PID* pid, float measured)//µ±Ç°Öµ
{
  pid->current = measured;
}


void RemainHover(float currentHeight, float targetHeight)
{	


				if(currentHeight<targetHeight)
					{
						THROTTLE=THROTTLE+0.0015f*(targetHeight-HCSR04_Distance);
				    if(THROTTLE>1620)
								THROTTLE=1620;//11.9//11.7:1500
					}

		
}

//½Ç¶È¿ØÖÆPID
void Roll_Pitch_Yaw_AnglePID(float Angle_roll,float Angle_pitch,float Angle_yaw,float Rate_yaw)
{
	float RateTarget ;//yaw_error;
	float IMU_Pitch, IMU_Roll, IMU_Yaw;
	float IMU_GYROx, IMU_GYROy, IMU_GYROz;

	IMU_GYROx = MPU6050_GYRO_LAST.X;
	IMU_GYROy = MPU6050_GYRO_LAST.Y;
	IMU_GYROz = MPU6050_GYRO_LAST.Z;
	IMU_Roll = Q_ANGLE.X;//ËÄÔªËØ·¨¼ÆËã³öÀ´µÄ½Ç¶È¡£
	IMU_Pitch = Q_ANGLE.Y;
	IMU_Yaw = Q_ANGLE.Z;

	//ROLL
	pidSetTarget(&Stabilize_Roll, Angle_roll);	 //Ä¿±ê½Ç¶È 40
	/**/
	RateTarget = pidUpdate(&Stabilize_Roll ,IMU_Roll , PID_dt);
	pidSetTarget(&RollRate, RateTarget);
//	/**/
	pidUpdate(&RollRate ,IMU_GYROx , PID_dt);

	RollRate.PID_out = Math_fConstrain(RollRate.PID_out,-300.0f,+300.0f);  //ÏÞÖÆ¿ØÖÆPWMÐÅºÅµÄ·ù¶È

//PITCH
	pidSetTarget(&Stabilize_Pitch, Angle_pitch);// Ä¿±ê½Ç¶È 40
	RateTarget = -pidUpdate(&Stabilize_Pitch ,IMU_Pitch , PID_dt);
	pidSetTarget(&PitchRate, RateTarget);
	pidUpdate(&PitchRate ,IMU_GYROy , PID_dt);
	PitchRate.PID_out = Math_fConstrain(PitchRate.PID_out,-300.0f,+300.0f);  //ÏÞÖÆ¿ØÖÆPWMÐÅºÅµÄ·ù¶È
	//YAW
	pidSetTarget(&Stabilize_Yaw, Angle_yaw);
	RateTarget = pidUpdate(&Stabilize_Yaw ,IMU_Yaw , PID_dt) + Rate_yaw;//Æ«º½½ÇµÄºÍroll pitchÓÐµã²î±ð
	pidSetTarget(&YawRate, RateTarget);
	pidUpdate(&YawRate ,IMU_GYROz , PID_dt);
	YawRate.PID_out = Math_fConstrain(YawRate.PID_out,-100.0f,+100.0f);  //ÏÞÖÆ¿ØÖÆPWMÐÅºÅµÄ·ù¶È	
	


}
//ÓÍÃÅ¿ØÖÆPID 
// TargetHigh  Ä¿±ê¸ß¶È£¬µ¥Î» cm
// isRate ¿ØÖÆ·½Ê½£¬1 Îª×èÄáÄ£Ê½	ÏÞÖÆÅÀÉýÂÊ
//                  0  ¶¨¸ßÄ£Ê½
float Climb_last_out = 0;
int16_t Auto_High_PID(float Target,uint8_t isRate)
{
	static float ClimbTarget = 0;
	static uint32_t last_time=0, now_time=0; // ²ÉÑùÖÜÆÚ¼ÆÊý µ¥Î» us
	now_time = micros();  //¶ÁÈ¡Ê±¼ä
	if(now_time > last_time)
	{
		ALT_Update_Interval =  ((float)(now_time - last_time) / 1000000.0f);
	}
	else
	{
		last_time = now_time;
		return Climb.PID_out;
	}	
	last_time = now_time;

	
		
	pidSetTarget(&AutoHigh_THR, Target);
	if(HCSR04_Update)
	{	
		ClimbTarget = pidUpdate(&AutoHigh_THR , HCSR04_Distance ,ALT_Update_Interval);
		HCSR04_Distance_Last = HCSR04_Distance	;
		HCSR04_Update = 0;
	}	

	ClimbTarget = Math_fConstrain(ClimbTarget,-50.0f,+50.0f); 
	
	
	pidSetTarget(&Climb, ClimbTarget);
	
	pidUpdate(&Climb ,IMU_SPEED_Z ,ALT_Update_Interval);
	Climb.PID_out = Math_fConstrain(Climb.PID_out,-300.0f,+300.0f);
	Climb.PID_out = Climb_last_out + Math_fConstrain((Climb.PID_out - Climb_last_out),-5,+5);
	Climb_last_out = Climb.PID_out; 
	

	
	return (int16_t)Climb.PID_out;
}

void Roll_Pitch_AccelPID(float Angle_Roll,float Angle_Pitch,float Angle_yaw,float Rate_yaw)
{
	
}

//-----------------------------------------------------------------------------
/// Low pass filter cut frequency for derivative calculation.
static const float Hold_filter = 7.9577e-3f;

float Position_Error[2];
float Position_Hold_i[2]={0.0f,0.0f};
float speed_error[2];
float speed_last_error[2]={0.0f,0.0f};
float Hold_last_error[2]={0.0f,0.0f};
float speed_d[2]={0.0f,0.0f};
float GPS_Hold_Angle[2]={0.0f,0.0f};
float Smooth_Ang[2]={0.0f,0.0f};

float Wrap_Ang(float ang)
{
  if (ang > 180)  ang -= 360;
  if (ang < -180) ang += 360;
  return ang;
}

//-----------------------------------------------------------------------------
void Position_Hold_Reset(void) 
{
	uint8_t	axis ;
	for (axis = 0; axis < 2; axis ++) 
	{
		Position_Hold_i[axis] = 0.0f;
		speed_last_error[axis] = 0.0f;
		Hold_last_error[axis] = 0.0f;
		speed_d[axis] = 0.0f;
		//speed_last_d[axis] = 0.0f;
		GPS_Hold_Angle[axis] = 0.0f;
		Smooth_Ang[axis] = 0.0f;
	}
	GPS_PITCH = 0.0f;
	GPS_ROLL = 0.0f;
}

//-----------------------------------------------------------------------------
void GPS_Position_Hold(float * Target_Lat,float * Target_Lon) 
{
	
}

int16_t Math_Constrain(int16_t value,int16_t min,int16_t max)
{
if(value > max)value = max;
	else if(value < min)value = min;
return value;
}

float Math_fConstrain(float value, float min, float max)//»ý·ÖÏîµÄ¸Ä½ø£ºÎªÁË¿Ë·þ»ý·Ö±¥ºÍ£¬´Ó¶ø²ÉÓÃÏÞÖÆ»ý·Ö·ù¶ÈµÄ·½·¨
{
if(value > max)value = max;
	else if(value < min)value = min;
return value;
}

int16_t Math_abs(int16_t value)
{
	if((value>0)||(value==0))
		return value;
	return -value;
}

int16_t Math_min(int16_t value1,int16_t value2)
{
	if(value1<value2)return value1;
	return value2;
}

int16_t Math_max(int16_t value1,int16_t value2)
{
	if(value1>value2)return value1;
	return value2;
}

float Get_Yaw_Error(float set,float currt)
{
	float temp;
	temp = set - currt;
	if(temp<-180) return (temp+360);
	else if(temp>180) return (temp-360);
	else return temp;
}

