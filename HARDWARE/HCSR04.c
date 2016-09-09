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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��
	
	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;  //TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	

	//��ʱ��TIM6��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM6�ж�,��������ж�
  TIM_UpdateRequestConfig(TIM6,TIM_UpdateSource_Regular);
	TIM6->CR1 |= 0x1<<2;//���ĸ�������Դ

//	TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx					 
}

void HCSR04_Init(void)
{
	HCSR04_GPIO_Config();
	TIM6_Int_Init(30000,83);	//��ʱ��2,1us�ж�
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//������ȼ�
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
			OSSemPost(Sem_Task_HCSR04_STOP);//�����ź��� 

}
	

//			if (HCSR04_Error == 0)
//			{
//				if(HCSR04_valid++>AVG_NUM+50)	 //����AVG_NUM+n�γ������߶���Ч����ôӦ��������������ѹ�߶ȵ�Ư��
//				{ 
//				//	MS561101_SetAlt(HCSR04_Distance);  //������ �߶���Ч���궨��ѹ�߶�
//					HCSR04_valid = AVG_NUM+48;
//					HCSR04_Update = 1;  //���������±�־
//					HCSR04_OK = 1;  //������������Ч��־
//					}
//			}
//			else
//			{
//				HCSR04_valid = 0;
//				HCSR04_OK = 0;  //������������Ч��־
//			}
		
	}
	OSIntExit();
}
void TIM6_DAC_IRQHandler(void)   //TIM5�ж�
{
  OSIntEnter();    
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  //���TIM6�����жϷ������
	{
		
		HCSR04_Error = 1;
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //���TIMx�����жϱ�־
	}
	OSIntExit();

}

void HCSR04_Get_Distance(int time)
{
	float Distance,Sound_Speed,x,y,z,IMU_SPEED_Z_tmp,IMU_SPEED_Z_sum;
	static uint32_t last_time=0, now_time=0; // �������ڼ��� ��λ us
	static float speed_Z[10],last_HCSR04_Distance=0;
	uint8_t i=0;
	//�����ڵ�ǰ�¶��� ��Ӧ�Ŀ����������Ĵ����ٶ�
	/*
	��������ʵ��ܶȺ͵��������йأ����Ҳ����ʵ��¶ȡ�
	ѹǿ��״̬�������ı䡣����������ÿ��Լ�����ף�
	���¶����߶�����0��ʱ����������Ϊ331��4�ף��룬
	15��ʱΪ340�ף��룬�¶�ÿ����1�棬����Լ����0��6�ף��롣
	*/
	Sound_Speed = 340.0f;//(332.0f+ (MS5611_Temperature/100.0f)*0.607f);
	Distance=(float)time/20000*Sound_Speed; //��λ����
	//ͨ���ռ���ά����任 ���Ǧֱ�߶�
	x=arm_sin_f32(-Pitch);
	y=-arm_sin_f32(-Roll)*arm_cos_f32(-Pitch);
	z=arm_cos_f32(-Roll)*arm_cos_f32(-Pitch);
	Distance*=z/sqrt(x*x+y*y+z*z);

	//HCSR04_NewDis(Distance);
	//HCSR04_Distance=HCSR04_GetAvg(Dis_buffer,AVG_NUM);
	if(HCSR04_Distance == 0) HCSR04_Distance_Last = Distance_KalmanFilter(Distance,0.004,0.04,0);
	
		now_time = micros();  //��ȡʱ�������õ���ʱ��7 
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
	if (HCSR04_Distance>400) HCSR04_Error = 1;   //��������
}
