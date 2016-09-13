#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_
#include "sys.h"

extern u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_GpsData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_Votage,Send_MotoPwm,Send_UserData;
extern int16_t UserData[20]; 


//void Nrf_Check_Event(void);
void Send_Data(void);
void Data_Receive_Anl(u8 *data_buf,u8 num);
void Data_Exchange(void);
void Data_Send_Status(void);	
void Data_Send_Senser(void);	
void Data_Send_RCData(void);	
void Data_Send_OFFSET(void);	
void Data_Send_PID1(void);
void Data_Send_PID2(void);
void Data_Send_PID3(void);
void Data_Send_PID4(void);
void Data_Send_PID5(void);
void Data_Send_PID6(void);
void Data_Send_MotoPWM(void);
void Data_Send_UserData(void);
void Data_Send_Check(u16 check);
void Respond_PID1(void);
void Respond_PID2(void);
void Respond_PID3(void);
void Data_Send_Check(u16 check);
void NRF_Send_Test(void);

void 	Nrf_Check_Event(void);
#endif
