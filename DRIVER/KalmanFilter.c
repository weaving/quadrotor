#include"kalmanfilter.h"
//Q系统过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值
//R测量过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值   
//如果你需要滤波结果更依赖于观测量，那就调小R，增大Q；反之，调大R，调小Q，这样估计值就取决于系统。
//如果R大Q小，就是说，状态估计值比测量值要可靠，这时，所得出的结果就是更接近估计值；
//如果R小Q大，这时，计算出来的结果就会更接近测量值。
/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/

float AccX_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}

float AccY_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}

float AccZ_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}

float Pressure_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}

float Altitude_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}

float Distance_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}
float GPS_SPEED_ROLL_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}
float GPS_SPEED_PITCH_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction)
{
	float R = MeasureNoise_R;
	float Q = ProcessNiose_Q;

	static	float x_last;

	float x_mid = x_last;
	float x_now;

	static	float p_last;

	float p_mid ;
	float p_now;
	float kg;	

	x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
	kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
	x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
		
	p_now=(1-kg)*p_mid;//最优值对应的covariance	

	p_last = p_now; //更新covariance值
	x_last = x_now; //更新系统状态值

	return x_now;		
}
