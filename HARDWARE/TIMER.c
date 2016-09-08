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
uint32_t time_cnt=0;
uint32_t time3_cnt=0;
void TIM9_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); //时钟使能
	
	//定时器TIM9初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE ); //使能指定的TIM9中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;  //TIM9中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM9, ENABLE);  //使能TIMx					 
}

void TIM4_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDT3 = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM9中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM4, ENABLE);  //使能TIMx					 
}

void TIM7_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //时钟使能
	
	//定时器TIM7初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  //TIM7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级5级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器


	TIM_Cmd(TIM7, ENABLE);  //使能TIMx					 
}

uint32_t micros(void)//利用定时器7做的
{
    uint32_t time_us;

    time_us = TIM_GetCounter(TIM7);

    return time_us + time_cnt*0xFFFF;
}

void TIM7_IRQHandler(void)   //TIM7中断
{
  OSIntEnter();    
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  //检查TIM7更新中断发生与否
	{
		time_cnt++;
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIMx更新中断标志
	}
	OSIntExit();
}

//void TIM4_IRQHandler(void)   //TIM4中断
//{
//	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)  //检查TIM4更新中断发生与否
//	{
//		time3_cnt++;
//		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志
//	}
//}
//定时器9中断服务程序
uint8_t Attitude;
void TIM1_BRK_TIM9_IRQHandler(void)   //TIM9中断
{
	static u16 ms1 = 0,ms2 = 0,ms3 = 0,ms4 = 0,ms5 = 0,ms6 = 0,ms7 = 0,ms8 = 0,ms9 = 0,ms10 = 0;				//中断次数计数器
	if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET)  //检查TIM9更新中断发生与否
		{
			TIM_ClearITPendingBit(TIM9, TIM_IT_Update  );  //清除TIMx更新中断标志
			ms1++;
			ms2++;
			ms3++;
			ms4++;
			ms5++;
			ms6++;
			ms7++;
			ms8++;
			ms9++;
			ms10++;
			if(ms1==2)			
			{
				ms1=0;
				MPU6050_Dataanl();
				
				Prepare_Data();
				
			}
			if(ms2==4)		
			{
				Get_Attitude();		//姿态计算
				Fly_Control();//阻尼爬升	
				Attitude=1;
				ms2=0;
			}
			if(ms3==10)
			{
				ms3=0;
//				MS5611_Run();							
			}
			if(ms6==20)		 
			{
				ms6=0;
				Send_Senser = 1;
				HMC5883L_Dataanl();
			}
		}
}










