#ifndef _KALMANFILTER_H
#define	_KALMANFILTER_H

float AccX_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float AccY_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float AccZ_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float Pressure_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float Altitude_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float Distance_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float GPS_SPEED_ROLL_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
float GPS_SPEED_PITCH_KalmanFilter(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,float InitialPrediction);
#endif


