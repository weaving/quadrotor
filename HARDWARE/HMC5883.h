#include "MPU6050.h"
#include "imu.h"

#ifndef __HMC5883_USER_CFG_H
#define __HMC5883_USER_CFG_H


extern uint16_t hmc5883_dat[3];

#define	HMC5883_I2C_Addr   0x3C	//磁场传感器器件地址   
#define MAG_DATA_REGISTER  0x03
#define HMC5883L_ConfigurationRegisterA  0x00
#define HMC5883L_ConfigurationRegisterB  0x01
#define HMC5883L_ModeRegister            0x02
#define HMC5883L_Output_X_MSB            0x03
#define HMC5883L_Output_X_LSB 					 0x04
#define HMC5883L_Output_Z_MSB            0x05
#define HMC5883L_Output_Z_LSB 					 0x06
#define HMC5883L_Output_Y_MSB            0x07
#define HMC5883L_Output_Y_LSB 					 0x08
#define HMC5883L_StatusRegister					 0x09
#define HMC5883L_ID_A										 0x0A
#define HMC5883L_ID_B 									 0x0B
#define HMC5883L_ID_C 									 0x0C


extern S_INT16_XYZ HMC5883L_LAST,AVG_MAG;
extern u8		   MAG_OFFSET_OK;
extern S_INT16_XYZ HMC5883L_OFFSET;
extern S_FLOAT_XYZ HMC5883L_Gain;
//extern S_FLOAT_XYZ HMC5883L_Gain;
extern float H_YAW;

void HMC5883_GET_DAT(void);
void HMC5883_Init(void);
uint16_t GetData_HMC5883(uint8_t REG_Address);
void HMC5883L_Dataanl(void);

void HMC5883L_Correction(void);
void HMC5883L_GetMaxMin(int16_t X,int16_t Y,int16_t Z);
void HMC5883L_GetOFFSET(void);
float HMC5883L_Calculate(void);
void GetMotoMag(void);
#endif
