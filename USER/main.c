#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "i2c.h"
#include "main.h"
#include "mpu6050.h"
#include "imu.h"
#include "hmc5883.h"
#include "data_transfer.h"
#include "timer.h"
#include "24l01.h"
#include "HCSR04.h"
#include "CONTROL.h"
#include "MOTO.h"
#include "UART.h"
//#include "arm_math.h"
#include "math.h"
#include "Flash_eeprom.h"
#include "pwm_capture.h"
#include "ov7670.h"
#include "exti.h"
#include "wft_controller.h"
#include "pid.h"
// STM32F407������ UCOSʵ��1
//UCOSII ��ֲ
//STM32F4���� 
//�Ա����̣�http://mcudev.taobao.com	
vs32				Alt;
//START ����
//�����������ȼ�
#define START_TASK_PRIO			30  ///��ʼ��������ȼ�Ϊ���
//���������ջ��С
#define START_STK_SIZE			128
//���������ջ
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//START ����
//�����������ȼ�
#define OV7670_TASK_PRIO		27  ///
//���������ջ��С
#define OV7670_STK_SIZE			64//+76800/4
//���������ջ
OS_STK OV7670_TASK_STK[OV7670_STK_SIZE];
//������
void ov7670_task(void *pdata);

//��̬��������
//�����������ȼ�
#define MPU6050_TASK_PRIO			7
//���������ջ��С
#define MPU6050_STK_SIZE				64
//�����ջ
OS_STK MPU6050_TASK_STK[MPU6050_STK_SIZE];
//������
void mpu6050_task(void *pdata);

//��̬��������
//�����������ȼ�
#define HMC5883_TASK_PRIO			8
//���������ջ��С
#define HMC5883_STK_SIZE				64*4
//�����ջ
OS_STK HMC5883_TASK_STK[HMC5883_STK_SIZE];
//������
void hmc5883_task(void *pdata);

//HCSR04�շ�����
//�����������ȼ�
#define HCSR04_TASK_PRIO			6
//���������ջ��С
#define HCSR04_STK_SIZE				64*2
//�����ջ
OS_STK HCSR04_TASK_STK[HCSR04_STK_SIZE];
//������
void hcsr04_task(void *pdata);


//NRF24L01�շ�����
//�����������ȼ�
#define NRF_TASK_PRIO			9
//���������ջ��С
#define NRF_STK_SIZE				64*6
//�����ջ
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
//������
void nrf_task(void *pdata);

//WFT07������
//�����������ȼ�
#define WFT_TASK_PRIO			10
//���������ջ��С
#define WFT_STK_SIZE				64
//�����ջ
OS_STK WFT_TASK_STK[WFT_STK_SIZE];
//������
void wft_task(void *pdata);

//WFT07������
//�����������ȼ�
#define SYSINFO_TASK_PRIO			29
//���������ջ��С
#define SYSINFO_STK_SIZE				64
//�����ջ
OS_STK SYSINFO_TASK_STK[SYSINFO_STK_SIZE];
//������
void sysinfo_task(void *pdata);

//LED0����
//�����������ȼ�
#define LED0_TASK_PRIO			24
//���������ջ��С
#define LED0_STK_SIZE				64
//�����ջ
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//������
void led0_task(void *pdata);

//LED1����
//�����������ȼ�
#define LED1_TASK_PRIO			25
//���������ջ��С
#define LED1_STK_SIZE				64
//�����ջ
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//������
void led1_task(void *pdata);

//�����������
#define FLOAT_TASK_PRIO		9
//���������ջ��С
#define FLOAT_STK_SIZE			128
//�����ջ
//���������ʹ��printf����ӡ�������ݵĻ�һ��Ҫ8�ֽڶ���
__align(8) OS_STK FLOAT_TASK_STK[FLOAT_STK_SIZE]; 
//������
void float_task(void *pdata);
void camera_refresh(void);
extern u8 ov_sta;	//��exit.c�� �涨��
extern OS_EVENT * Sem_Task_HCSR04_START;
extern OS_EVENT * Sem_Task_HCSR04_STOP;
extern uint16_t hcsr04_time;
int main(void)
{
#if SUPPORT_OV7670==1
	u8 lightmode=0,saturation=2,brightness=2,contrast=2;
	u8 effect=0;	
#endif
	
	delay_init(168);       //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
#if EN_USART1_RX 
	uart_init(115200);    //���ڲ���������
#endif
	
	LED_Init();  //LED��ʼ�� PA6 PA7��ӦLED
	PWM_Capture_Init(0xFFFF,83);  //1MHZ���� TIM4 TIM5 ������׽ң����8ͨ������
	PWM_MOTO_Config(); //TIM3 �������� PWM��·���
	TIM_Cmd(TIM3, ENABLE);
#if SUPPORT_OV7670==1
	OV7670_Init();
	OV7670_Light_Mode(lightmode);
	OV7670_Color_Saturation(saturation);
	OV7670_Brightness(brightness);
	OV7670_Contrast(contrast);
 	OV7670_Special_Effects(effect);	
	OV7670_Window_Set(12,174,240,320);
#endif

	MPU6050_Init();
	HMC5883_Init();
	NRF24L01_Init();
	Control_Init();
//	HCSR04_Init();
	TIM7_Int_Init(0xFFFF,83);//1Mhz�ļ���Ƶ��,1usʱ�����	
  GPIO_ResetBits(GPIOA,GPIO_Pin_6 );//������ʾ�����ʼ��û����
	
	OSInit();  //UCOS��ʼ��
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO); //������ʼ����
	OSStart(); //��ʼ����
}

//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
	Sem_Task_HCSR04_START    = OSSemCreate(0);
	Sem_Task_HCSR04_STOP    = OSSemCreate(0);	
	OSStatInit();  //����ͳ������
	
	OS_ENTER_CRITICAL();  //�����ٽ���(�ر��ж�)
	OSTaskCreate(mpu6050_task,(void*)0,(OS_STK*)&MPU6050_TASK_STK[MPU6050_STK_SIZE-1],MPU6050_TASK_PRIO);//����MPU6050����
	OSTaskCreate(hmc5883_task,(void*)0,(OS_STK*)&HMC5883_TASK_STK[HMC5883_STK_SIZE-1],HMC5883_TASK_PRIO);//����HMC5883����

//	OSTaskCreate(led0_task,(void*)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);//����LED0����
//	OSTaskCreate(led1_task,(void*)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);//����LED1����
//	OSTaskCreate(float_task,(void*)0,(OS_STK*)&FLOAT_TASK_STK[FLOAT_STK_SIZE-1],FLOAT_TASK_PRIO);//���������������
//	OSTaskCreate(hcsr04_task,(void*)0,(OS_STK*)&HCSR04_TASK_STK[HCSR04_STK_SIZE-1],HCSR04_TASK_PRIO);//����HCSR04����
	OSTaskCreate(wft_task,(void*)0,(OS_STK*)&WFT_TASK_STK[WFT_STK_SIZE-1],WFT_TASK_PRIO);//����WFT07����

	OSTaskCreate(nrf_task,(void*)0,(OS_STK*)&NRF_TASK_STK[NRF_STK_SIZE-1],NRF_TASK_PRIO);//����NRF����
#if SUPPORT_OV7670==1
		OSTaskCreate(ov7670_task,(void*)0,(OS_STK*)&OV7670_TASK_STK[OV7670_STK_SIZE-1],OV7670_TASK_PRIO);//����NRF����
#endif
//	OSTaskCreate(sysinfo_task,(void*)0,(OS_STK*)&SYSINFO_TASK_STK[SYSINFO_STK_SIZE-1],SYSINFO_TASK_PRIO);//����NRF����
	OSTaskSuspend(START_TASK_PRIO);//����ʼ����
	OS_EXIT_CRITICAL();  //�˳��ٽ���(���ж�)
}
 

//mpu6050
void mpu6050_task(void *pdata)
{
	
//	OS_CPU_SR cpu_sr=0;
	u8 ms1=0;
	while(1)
	{
//			time1 = OSTimeGet();
			if(ms1==0||ms1==1)
			{
				MPU6050_Dataanl();
				Prepare_Data();
				Send_Status=1;
				ms1++;
			}
			if(ms1==2)
			{
				Get_Attitude();
				Fly_Control();
				ms1=0;
			}
			delay_ms(3);
//time1 = OSTimeGet();
//		
//			printf(" CPU Usage: %ld%    \r\n",OSCPUUsage);  
//					time2 = OSTimeGet();
//		  time = time2-time1;
//					OS_ENTER_CRITICAL();	
//			printf(" TIME Usage: %ld%    \r\n",time2-time1);  
//			OS_EXIT_CRITICAL();	
	};
}

// hmc5883
void hmc5883_task(void *pdata)
{
	while(1)
	{
			HMC5883L_Dataanl();
			delay_ms(20);
	};
}
//hcsr04
void hcsr04_task(void *pdata)
{	
	INT8U err;
	INT16U timeout;
	while(1)
	{
		HCSR04_Run();
		OSSemPend(Sem_Task_HCSR04_STOP,timeout,&err);
		HCSR04_Get_Distance(hcsr04_time);
		delay_ms(30);
	};
}

//nrf
void nrf_task(void *pdata)
{
	while(NRF24L01_Check()){};
	NRF24L01_RX_Mode();		
	while(1)
	{
//	Send_Data();

	
	UserData[8] = RCTarget.Throttle;

		Data_Send_UserData();

//	Data_Send_UserData();
//  Data_Send_Senser();
//		Data_Send_Status();
		delay_ms(30);
	};
}

//ov7670
void ov7670_task(void *pdata)
{

//	OV7670_Window_Set(12,176,240,320);
	OV7670_CS=0;	

	EXTI9_Init();
	
	while(1)
	{


//		camera_refresh();
//		delay_ms(50);
	}
}

//wft07 
void wft_task(void *pdata)
{


	while(1)
	{
		Pwm_In_Convert();
		WFT_CheckLock();
		
//  	TIM3->CCR1 = RCTarget.Throttle;
//		TIM3->CCR2 = RCTarget.Throttle;
//		TIM3->CCR3 = RCTarget.Throttle;
//		TIM3->CCR4 = RCTarget.Throttle;

		delay_ms(50);
	}
}

//sysinfo
void sysinfo_task(void *pdata)
{
	while(1)
	{
		printf(" CPU Usage: %ld%    \r\n",OSCPUUsage);  
		delay_ms(200);
	}
}
//LED0����
void led0_task(void *pdata)
{
	while(1)
	{
		LED0=0;
		delay_ms(500);

		LED0=1;
		delay_ms(500);
	};
}

//LED1����
void led1_task(void *pdata)
{
	u32 i,j;
	while(1)
	{
		LED1=0;
//		delay_ms(500);
				for(j=0;j<200;j++)
		for(i=0;i<2000000;i++);
		LED1=1;
//		delay_ms(500);
					for(j=0;j<200;j++)
		for(i=0;i<2000000;i++);
		

	};
}

//�����������
void float_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_ENTER_CRITICAL();	//���ж�
		printf("float_num��ֵΪ: %f\r\n",float_num); //���ڴ�ӡ���
		OS_EXIT_CRITICAL();		//���ж�
		delay_ms(100);
	}
}

void camera_refresh(void)
{
	u32 i,j;
 	u16 color;
	OS_CPU_SR cpu_sr=0;
	INT32U time1,time2 ;
	if(ov_sta)
	{
time1 = OSTimeGet();

		OV7670_RRST=0;				//��ʼ��λ��ָ�� 
		OV7670_RCK=0;
		OV7670_RCK=1;
		OV7670_RCK=0;
		OV7670_RRST=1;				//��λ��ָ����� 
		OV7670_RCK=1;  
		for(j=0;j<76800;j++)
		{
			OV7670_RCK=0;
			color=GPIOD->IDR&0XFF;	//������
			OV7670_RCK=1; 
			color<<=8;  
			OV7670_RCK=0;
			color|=GPIOD->IDR&0XFF;	//������
			OV7670_RCK=1;
//		printf("%d ",color);	
			if(j>76800/2) continue;
			if(j%320==0)i++;
//	   	ov7670_data[j%38000]= color;
//			if(j==10)
//				;
		}   
			time2 = OSTimeGet();
		OS_ENTER_CRITICAL();
			printf(" TIME Usage: %ld%    \r\n",time2-time1);  
		OS_EXIT_CRITICAL();
//		EXTI->PR=1<<9;     			//���LINE9�ϵ��жϱ�־λ
		ov_sta=0;					//��ʼ��һ�βɼ�
		//	ov_frame++; 
	} 
}	
