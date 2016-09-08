#ifndef __PWM_H
#define __PWM_H 	

#include "stm32f4xx.h"
#include "delay.h"
#define Moto_PwmMax 2000
#define Moto_PwmMin 1000
/**
  * @brief  ��������������� PWM��
            ʹ��     PC6��PC7��PC8��PC9
			�ֱ��Ӧ OC1��OC2��OC3��OC4
			
			ʹ�÷�����
			�ȵ���TIM_Config();
			��Ҫʹ�õ�ʱ�� ����TIM3��ʱ��
			�ٵ���Electronic_Speed_Controller_init();
	        Ȼ��Ϳ��Լ����ţ����������ת
  * @param  None
  * @retval None
  */
  
/* TIM3 �������4·PWM */
void TIM_PWM_Config(void);
//TIM_Cmd(TIM3, ENABLE);
void PWM_MOTO_Normal_Config(void);
void Electronic_Speed_Controller_init(void);
void Electronic_Speed_Controller_test(void);
void Moto_Reflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
#endif
