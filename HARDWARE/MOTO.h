#ifndef __PWM_H
#define __PWM_H 	

#include "stm32f4xx.h"
#include "delay.h"
#define Moto_PwmMax 2000
#define Moto_PwmMin 1000
/**
  * @brief  这个函数用来配置 PWM波
            使用     PC6、PC7、PC8、PC9
			分别对应 OC1、OC2、OC3、OC4
			
			使用方法：
			先调用TIM_Config();
			需要使用的时候 开启TIM3定时器
			再调用Electronic_Speed_Controller_init();
	        然后就可以加油门，电机可以旋转
  * @param  None
  * @retval None
  */
  
/* TIM3 用来输出4路PWM */
void TIM_PWM_Config(void);
//TIM_Cmd(TIM3, ENABLE);
void PWM_MOTO_Normal_Config(void);
void Electronic_Speed_Controller_init(void);
void Electronic_Speed_Controller_test(void);
void Moto_Reflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
#endif
