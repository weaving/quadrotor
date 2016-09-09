#include "stm32f4xx.h"
#include "HCSR04.h"
#include "imu.h"
#include "data_transfer.h"
#include "kalmanfilter.h"
#include "math.h"
#include "sys.h"
#include "includes.h"
#include "arm_math.h"
#include "timer.h"
#define AVG_NUM 5
#define Distance_Offset 12.1f
float old_HCSR04_Distance,V;
float HCSR04_Distance,HCSR04_Distance_Last;
float Dis_buffer[AVG_NUM];
uint8_t HCSR04_valid,HCSR04_RunFlag = 1,HCSR04_Error = 1,HCSR04_OK = 0,Dis_index = 0,HCSR04_Update = 0;

OS_EVENT * Sem_Task_HCSR04_START;
OS_EVENT * Sem_Task_HCSR04_STOP;
uint16_t hcsr04_time;
void HCSR04_EXTI_Configuration(void);
void HCSR04_GPIO_Config(void);



void TIM6_Int_Init(u16 arr,u16 psc)
{
	//uint16_t PrescalerValue = 0;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;  //TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	

	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM6中断,允许更新中断
  TIM_UpdateRequestConfig(TIM6,TIM_UpdateSource_Regular);
	TIM6->CR1 |= 0x1<<2;//更改更新请求源

//	TIM_Cmd(TIM6, ENABLE);  //使能TIMx					 
}

void HCSR04_Init(void)
{
	HCSR04_GPIO_Config();
	TIM6_Int_Init(30000,83);	//定时器2,1us中断
	HCSR04_EXTI_Configuration();
}

void HCSR04_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}
void HCSR04_EXTI_Configuration(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOE clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure PD.14 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  /* Connect EXTI14 Line to PE.14 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource14);
	
  /* Configure EXTI14 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line14;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
 
  /* Enable and set EXTI9_5 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//最高优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
}



void HCSR04_Run(void)
{
	GPIO_SetBits(GPIOE,GPIO_Pin_15);
	delay_us(12);
	GPIO_ResetBits(GPIOE,GPIO_Pin_15);

}


		uint16_t t;
void EXTI15_10_IRQHandler(void)
{

  OSIntEnter();    
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		
		EXTI_ClearITPendingBit(EXTI_Line14);
	
		if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14)==1)
		{
			HCSR04_Error = 0;
			TIM_SetCounter(TIM6,0);
			TIM_Cmd(TIM6, ENABLE);

		}
		else if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14)==0)
		{		
			TIM_Cmd(TIM6, DISABLE);
			hcsr04_time = TIM_GetCounter(TIM6);
			HCSR04_Update = 1; 
			OSSemPost(Sem_Task_HCSR04_STOP);//发送信号量 

}
	

//			if (HCSR04_Error == 0)
//			{
//				if(HCSR04_valid++>AVG_NUM+50)	 //连续AVG_NUM+n次超声波高度有效，那么应该用它来修正气压高度的漂移
//				{ 
//				//	MS561101_SetAlt(HCSR04_Distance);  //超声波 高度有效。标定气压高度
//					HCSR04_valid = AVG_NUM+48;
//					HCSR04_Update = 1;  //超声波更新标志
//					HCSR04_OK = 1;  //超声波数据有效标志
//					}
//			}
//			else
//			{
//				HCSR04_valid = 0;
//				HCSR04_OK = 0;  //超声波数据有效标志
//			}
		
	}
	OSIntExit();
}
void TIM6_DAC_IRQHandler(void)   //TIM5中断
{
  OSIntEnter();    
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  //检查TIM6更新中断发生与否
	{
		
		HCSR04_Error = 1;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //清除TIMx更新中断标志
	}
	OSIntExit();

}

void HCSR04_Get_Distance(int time)
{
	float Distance,Sound_Speed,x,y,z,IMU_SPEED_Z_tmp,IMU_SPEED_Z_sum;
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	static float speed_Z[10],last_HCSR04_Distance=0;
	uint8_t i=0;
	//计算在当前温度下 对应的空气中声音的传播速度
	/*
	音速与介质的密度和弹性性质有关，因此也随介质的温度、
	压强等状态参量而改变。气体中音速每秒约数百米，
	随温度升高而增大，0℃时空气中音速为331．4米／秒，
	15℃时为340米／秒，温度每升高1℃，音速约增加0．6米／秒。
	*/
	Sound_Speed = 340.0f;//(332.0f+ (MS5611_Temperature/100.0f)*0.607f);
	Distance=(float)time/20000*Sound_Speed; //单位厘米
	//通过空间三维坐标变换 求出铅直高度
	x=arm_sin_f32(-Pitch);
	y=-arm_sin_f32(-Roll)*arm_cos_f32(-Pitch);
	z=arm_cos_f32(-Roll)*arm_cos_f32(-Pitch);
	Distance*=z/sqrt(x*x+y*y+z*z);

	//HCSR04_NewDis(Distance);
	//HCSR04_Distance=HCSR04_GetAvg(Dis_buffer,AVG_NUM);
	if(HCSR04_Distance == 0) HCSR04_Distance_Last = Distance_KalmanFilter(Distance,0.004,0.04,0);
	
		now_time = micros();  //读取时间这里用到定时器7 
	if(now_time < last_time)
	{
		last_time = now_time;
		return;
	}	
	
	HCSR04_Distance = Distance_KalmanFilter(Distance,0.004,0.04,0);
	IMU_SPEED_Z_tmp = (HCSR04_Distance-last_HCSR04_Distance)/(now_time-last_time)*1000000.0f;
	/* cal speed_Z */
	/* sum speed_Z 10 times */
  for(i=0;i<sizeof(speed_Z)/sizeof(float);i++)
	  speed_Z[i]=speed_Z[i+1];
	speed_Z[i] = 	IMU_SPEED_Z_tmp;
	
	/* then cal the average speed_Z */
	IMU_SPEED_Z_sum=0;
	for(i=0;i<sizeof(speed_Z)/sizeof(float);i++)
		IMU_SPEED_Z_sum += speed_Z[i];
	IMU_SPEED_Z = IMU_SPEED_Z_sum/(sizeof(speed_Z)/sizeof(float));
	
	/* ready for the next cal */
	last_time = now_time;
	last_HCSR04_Distance = HCSR04_Distance;
	UserData[3] = IMU_SPEED_Z;
	if (HCSR04_Distance>400) HCSR04_Error = 1;   //超出量程
}
