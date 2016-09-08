#ifndef _HCSR04_H
#define _HCSR04_H
#include "delay.h"
#include "sys.h"
#include "includes.h"
/**
  * @brief  这个函数用来配置 超声波传感器
			
			使用方法：
			先调用HCSR04_Init();
			然后调用HCSR04_Run();
			每隔60ms 检测一次，这个间隔时间在 HCSR04_Run(void);里设置
  * @param  None
  * @retval None
  */
  

extern float HCSR04_Distance,HCSR04_Distance_Last;
extern uint8_t HCSR04_Error,HCSR04_OK,HCSR04_RunFlag,HCSR04_Update;
extern OS_EVENT * Sem_Task_HCSR04;

void HCSR04_Get_Distance(int time);
void HCSR04_GPIO_Configuration(void);
void HCSR04_EXTI_Configuration(void);
void TIM6_Int_Init(u16 arr,u16 psc);

void HCSR04_Init(void);
void HCSR04_Run(void);
void HCSR04_Calalute(void);


#endif
