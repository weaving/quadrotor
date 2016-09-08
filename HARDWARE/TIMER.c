#include "TIMER.h"
#include "UART.h"
#include "data_transfer.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "ms5611.h"
#include "imu.h"
#include "control.h"
#include "includes.h"
#include "sys.h"
static uint32_t time_cnt=0;

void TIM7_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM7��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  //TIM7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�5��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM7, ENABLE);  //ʹ��TIMx					 
}

uint32_t micros(void)//���ö�ʱ��7����
{
    uint32_t time_us;

    time_us = TIM_GetCounter(TIM7);

    return time_us + time_cnt*0xFFFF;
}

void TIM7_IRQHandler(void)   //TIM7�ж�
{
  OSIntEnter();    
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  //���TIM7�����жϷ������
	{
		time_cnt++;
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIMx�����жϱ�־
	}
	OSIntExit();
}

