#include "ms5611.h"
#include "timer.h"
#include "kalmanfilter.h"
#include "data_transfer.h"
#include "delay.h"
#include <math.h>

#define AVG_NUM 80

//单位 [温度 0.01度] [气压 帕]  [高度0.01厘米] 
float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_AltRate;
float Temp_buffer[AVG_NUM],Press_buffer[AVG_NUM],Alt_buffer[AVG_NUM];
float Alt_Offset_Pa=0,Alt_Offset_cm=0; //存放着0米时 对应的气压值  这个值存放上电时的气压值 
uint16_t  Covert_count=0;
uint8_t ALT_Updated = 0; //气压计高度更新完成标志。

uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_prom[7];  // on-chip ROM
float PRESS_BUF[AVG_NUM];

void ms5611_reset(void)
{
    MS5611_SendCommand(MS5611_ADDR, MS561101_RST);
	Delay_1ms(100);
}
void ms5611_read_prom(void)
{
    u8 i;
	uint16_t rxbuf2;
	uint8_t rxbuf[2] = { 0, 0 };
	for (i=0;i<=6;i++)
	{

//		rxbuf[0] = I2C_ByteRead(MS5611_ADDR,MS561101_PROM_RD+i*2);
//		rxbuf[1] = I2C_ByteRead(MS5611_ADDR,MS561101_PROM_RD+i*2+1);
		
		ms5611_prom[i]  = MS5611_ReadProm(MS5611_ADDR,MS561101_PROM_RD+i*2);
		
//		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}
}
uint32_t ms5611_read_adc(void)
{
	uint32_t rxbuf;
	rxbuf = MS5611_ReadADC(MS5611_ADDR,MS561101_ADC_RD);


	
	return rxbuf;
}
void ms5611_start_ut(void)
{
	MS5611_SendCommand(MS5611_ADDR,MS561101_D2_OSR_4096); // D2 (temperature) conversion start!
}
void ms5611_get_ut(void)
{
    ms5611_ut = ms5611_read_adc();
}
void ms5611_start_up(void)
{
	MS5611_SendCommand(MS5611_ADDR,MS561101_D1_OSR_4096); // D1 (pressure)  conversion start!
}
void ms5611_get_up(void)
{
    ms5611_up = ms5611_read_adc();
}

// FIFO 队列
float Temp_buffer[AVG_NUM],Press_buffer[AVG_NUM],Alt_buffer[AVG_NUM];
uint8_t temp_index=0,press_index=0,alt_index=0; //队列指针

//添加一个新的值到 温度队列 进行滤波
void MS561101_NewTemp(float val) 
{
  Temp_buffer[temp_index] = val;
  temp_index = (temp_index + 1) % AVG_NUM;
}

//添加一个新的值到 气压队列 进行滤波
void MS561101_NewPress(float val)
 {
  Press_buffer[press_index] = val;
  press_index = (press_index + 1) % AVG_NUM;
}

//添加一个新的值到 高度队列 进行滤波
void MS561101_NewAlt(float val)
 {
  int16_t i;
  for(i=1;i<AVG_NUM;i++)
  Alt_buffer[i-1] = Alt_buffer[i];
  Alt_buffer[AVG_NUM-1] = val;
}

//取气压计的D变化率
float MS5611_Get_D(void)
{
	float new=0,old=0;
	int16_t i;
	for(i=0;i<AVG_NUM/2;i++)
		old += Alt_buffer[i];
	old /= (AVG_NUM/2);

	for(i=AVG_NUM/2;i<AVG_NUM;i++)
	    new += Alt_buffer[i];
	new /= (AVG_NUM/2);

	return new - old;
}

//读取队列 的平均值
float MS561101_getAvg(float * buff, int size)
 {
  float sum = 0.0;
  int i;
  for(i=0; i<size; i++) {
    sum += buff[i];
  }
  return sum / size;
}

/**************************实现函数********************************************
*函数原型:		float MS561101_get_altitude(void)
*功　　能:	    将当前的气压值转成 高度。	 
*******************************************************************************/
float MS561101_get_altitude(void)
{

	float Altitude;
	if(Alt_Offset_Pa==0){ // 是否初始化过0米气压值？
		if(Covert_count++<250);  //等待气压稳定 后 再取零米时的气压值
		else Alt_Offset_Pa = MS5611_Pressure; //把 当前气压值保存成 0 米时的气压
		Altitude = 0; //高度 为 0
		return Altitude;
	}
	//计算相对于 上电时的位置的 高度值 。
	Altitude = 4433000.0f * (1.0f - pow((MS5611_Pressure / Alt_Offset_Pa), 0.190295f));
	//Altitude = Altitude_KalmanFilter(Altitude,0.008,30,0);
	Altitude += Alt_Offset_cm ;  //加偏置
	//MS561101_NewAlt(Altitude);
	//MS561101_getAvg(Alt_buffer,AVG_NUM);
	return (Altitude);
}
void ms5611_calculate(void)
{
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	float fullT;
	static float start_time=0;

    int32_t temperature, off2 = 0, sens2 = 0, delt, dT;
    int32_t pressure = 0;
	int64_t off,sens;

	now_time = micros();  //读取时间
//	if(now_time > last_time)
//	{
//		fullT =  ((float)(now_time - last_time) / 2000000.0f);
//	}
//	else
//	{
//		last_time = now_time;
//		return;
//	}	
//	last_time = now_time;

    dT = ms5611_ut - ((uint32_t)ms5611_prom[4] << 8);
    off = ((uint32_t)ms5611_prom[1] << 16) + (((int64_t)dT * ms5611_prom[3]) >> 7);
    sens = ((uint32_t)ms5611_prom[0] << 15) + (((int64_t)dT * ms5611_prom[2]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[5]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
	//UserData[6] = (int)((pressure-100000)*10);
	//MS561101_NewPress(pressure);
    //MS5611_Pressure = MS561101_getAvg(Press_buffer,AVG_NUM); //0.01mbar
	if(start_time<3)	  //让数据快速稳定
	{
		start_time+= fullT;
		MS5611_Pressure = Pressure_KalmanFilter((float)pressure,0.1,25,0);
	}	
	else
	{
		MS5611_Pressure = Pressure_KalmanFilter((float)pressure,0.04,25,0);	
	}


	//UserData[7] = (int)((MS5611_Pressure-100000)*10);

	MS561101_NewTemp(temperature);
	MS5611_Temperature = MS561101_getAvg(Temp_buffer,AVG_NUM); //0.01c

	MS5611_Altitude = MS561101_get_altitude(); //  cm
	MS561101_NewAlt(MS5611_Altitude);
	MS5611_AltRate = MS5611_Get_D() / (fullT * AVG_NUM);
	ALT_Updated = 1; //高度更新 完成
}
/**************************实现函数********************************************
*函数原型:		void MS561101_ResetAlt(void)
*功　　能:	    将当前的气压做为0米时的气压。	 
*******************************************************************************/
void MS561101_ResetAlt(void)
{
	Alt_Offset_Pa = MS5611_Pressure; //把 当前气压值保存成 0 米时的气压	
	Alt_Offset_cm = 0;
}

/**************************实现函数********************************************
*函数原型:		void MS561101_SetAlt(void)
*功　　能:	    将当前的气压做为 Current 米时的气压。	 
*******************************************************************************/
void MS561101_SetAlt(float Current)
{
	Alt_Offset_Pa = MS5611_Pressure; //把 当前气压值保存成 0 米时的气压	
	Alt_Offset_cm = Current; //米转成 CM
	MS561101_NewAlt(Current);	 //新的高度值
	ALT_Updated = 1; //高度更新 完成。
}
void MS5611_Run(void)
{
	static u8 state=0;
	
	switch(state)
	{
		case 0:	ms5611_reset();
						state++;
						break;
		case 1: ms5611_read_prom();
						state++;
						break;
		case 2:	ms5611_start_ut();
						state++;
						break;
		case 3:	ms5611_get_ut();
						ms5611_start_up();
						state++;
						break;
		case 4:	ms5611_get_up();
						ms5611_calculate();
						ms5611_start_ut();
						state=3;
						break;
		default:ms5611_reset();
						state=1;
						break;
	}
}
