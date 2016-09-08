#include"kalmanfilter.h"
//Qϵͳ���̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ
//R�������̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ   
//�������Ҫ�˲�����������ڹ۲������Ǿ͵�СR������Q����֮������R����СQ����������ֵ��ȡ����ϵͳ��
//���R��QС������˵��״̬����ֵ�Ȳ���ֵҪ�ɿ�����ʱ�����ó��Ľ�����Ǹ��ӽ�����ֵ��
//���RСQ����ʱ����������Ľ���ͻ���ӽ�����ֵ��
/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

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
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
	kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
	x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
		
	p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance	

	p_last = p_now; //����covarianceֵ
	x_last = x_now; //����ϵͳ״ֵ̬

	return x_now;		
}
