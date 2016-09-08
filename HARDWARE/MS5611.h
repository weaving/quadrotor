#ifndef _MS5611_H_
#define _MS5611_H_
#include "i2c.h"
#include "sys.h"

#define MS5611_ADDR             0xee 
#define MS561101_D1      0x40 
#define MS561101_D2      0x50 
#define MS561101_RST     0x1E   
#define MS561101_D1_OSR_256    0x40 
#define MS561101_D1_OSR_512    0x42 
#define MS561101_D1_OSR_1024   0x44 
#define MS561101_D1_OSR_2048   0x46 
#define MS561101_D1_OSR_4096   0x48   
#define MS561101_D2_OSR_256    0x50 
#define MS561101_D2_OSR_512    0x52 

#define MS561101_D2_OSR_1024   0x54 
#define MS561101_D2_OSR_2048   0x56 
#define MS561101_D2_OSR_4096   0x58   
#define MS561101_ADC_RD     0x00 
#define MS561101_PROM_RD    0xA0 
#define MS561101_PROM_CRC   0xAE

extern float Alt_Offset_Pa,Alt_Offset_cm;
extern uint32_t ms5611_ut;  // static result of temperature measurement
extern uint32_t ms5611_up;  // static result of pressure measurement

extern float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_AltRate;
extern uint8_t ALT_Updated;

void MS5611_Run(void);
void MS561101_NewTemp(float val);
void MS561101_NewPress(float val);
void MS561101_NewAlt(float val);
float MS5611_Get_D(void);
float MS561101_getAvg(float * buff, int size);
float MS561101_get_altitude(void);
void MS561101_ResetAlt(void);
void MS561101_SetAlt(float Current);
void ms5611_reset(void);
void ms5611_read_prom(void);
uint32_t ms5611_read_adc(void);
void ms5611_start_ut(void);
void ms5611_get_ut(void);  
void ms5611_start_up(void); 
void ms5611_get_up(void);
void ms5611_calculate(void);
#endif
