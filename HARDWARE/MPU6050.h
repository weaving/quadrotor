#ifndef _MPU6050_H
#define _MPU6050_H
#include "stm32f4xx.h"

//****************************************
// ���� MPU6050�ڲ���ַ
//****************************************
#define SMPLRT_DIV 0x19 //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define MPU6050_CONFIG 0x1A //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG 0x1B //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define ACCEL_CONFIG 0x1C //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define I2C_MST_CTRL 0x24 //10000000=0x80
#define INT_PIN_CFG  0x37  //00110010=0x32
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B //��Դ����������ֵ��0x00(��������)
#define WHO_AM_I 0x75 //IIC ��ַ�Ĵ���(Ĭ����ֵ 0x68��ֻ��)
#define MPU6050_I2C_Addr 0xD0 //IIC д��ʱ�ĵ�ַ�ֽ����ݣ�+1 Ϊ��ȡ                      

typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;
extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡֵ
extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//��Ư
extern u8							GYRO_OFFSET_OK;
extern u8							ACC_OFFSET_OK;

void MPU6050_Init(void);
void MPU6050_Dataanl(void);
uint16_t GetData_MPU6050(uint8_t REG_Address);

#endif  