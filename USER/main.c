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
#include "OpticalFlow.h"
#include "iwdg.h"

// STM32F407开发板 UCOS实验1
//UCOSII 移植
//STM32F4工程 
//淘宝店铺：http://mcudev.taobao.com	
vs32				Alt;
//START 任务
//设置任务优先级
#define START_TASK_PRIO			30  ///开始任务的优先级为最低
//设置任务堆栈大小
#define START_STK_SIZE			128
//任务任务堆栈
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//START 任务
//设置任务优先级
#define OV7670_TASK_PRIO		27  ///
//设置任务堆栈大小
#define OV7670_STK_SIZE			64//+76800/4
//任务任务堆栈
OS_STK OV7670_TASK_STK[OV7670_STK_SIZE];
//任务函数
void ov7670_task(void *pdata);

//姿态解算任务
//设置任务优先级
#define MPU6050_TASK_PRIO			7
//设置任务堆栈大小
#define MPU6050_STK_SIZE				64
//任务堆栈
OS_STK MPU6050_TASK_STK[MPU6050_STK_SIZE];
//任务函数
void mpu6050_task(void *pdata);

//姿态解算任务
//设置任务优先级
#define HMC5883_TASK_PRIO			8
//设置任务堆栈大小
#define HMC5883_STK_SIZE				64*4
//任务堆栈
OS_STK HMC5883_TASK_STK[HMC5883_STK_SIZE];
//任务函数
void hmc5883_task(void *pdata);

//HCSR04收发数据
//设置任务优先级
#define HCSR04_TASK_PRIO			6
//设置任务堆栈大小
#define HCSR04_STK_SIZE				64*3
//任务堆栈
OS_STK HCSR04_TASK_STK[HCSR04_STK_SIZE];
//任务函数
void hcsr04_task(void *pdata);


//光流传感器
//设置任务优先级
#define OpticalFlow_TASK_PRIO			5
//设置任务堆栈大小
#define OpticalFlow_STK_SIZE				64*200
//任务堆栈
OS_STK OpticalFlow_TASK_STK[OpticalFlow_STK_SIZE];
//任务函数
void OpticalFlow_task(void *pdata);


//NRF24L01收发数据
//设置任务优先级
#define NRF_TASK_PRIO			9
//设置任务堆栈大小
#define NRF_STK_SIZE				64*6
//任务堆栈
OS_STK NRF_TASK_STK[NRF_STK_SIZE];
//任务函数
void nrf_task(void *pdata);

//WFT07发数据
//设置任务优先级
#define WFT_TASK_PRIO			10
//设置任务堆栈大小
#define WFT_STK_SIZE				64
//任务堆栈
OS_STK WFT_TASK_STK[WFT_STK_SIZE];
//任务函数
void wft_task(void *pdata);

//WFT07发数据
//设置任务优先级
#define SYSINFO_TASK_PRIO			29
//设置任务堆栈大小
#define SYSINFO_STK_SIZE				64
//任务堆栈
OS_STK SYSINFO_TASK_STK[SYSINFO_STK_SIZE];
//任务函数
void sysinfo_task(void *pdata);

//LED0任务
//设置任务优先级
#define LED0_TASK_PRIO			24
//设置任务堆栈大小
#define LED0_STK_SIZE				64
//任务堆栈
OS_STK LED0_TASK_STK[LED0_STK_SIZE];
//任务函数
void led0_task(void *pdata);

//LED1任务
//设置任务优先级
#define LED1_TASK_PRIO			25
//设置任务堆栈大小
#define LED1_STK_SIZE				64
//任务堆栈
OS_STK LED1_TASK_STK[LED1_STK_SIZE];
//任务函数
void led1_task(void *pdata);

//浮点测试任务
#define FLOAT_TASK_PRIO		9
//设置任务堆栈大小
#define FLOAT_STK_SIZE			128
//任务堆栈
//如果任务中使用printf来打印浮点数据的话一点要8字节对齐
__align(8) OS_STK FLOAT_TASK_STK[FLOAT_STK_SIZE]; 
//任务函数
void float_task(void *pdata);
void camera_refresh(void);
extern u8 ov_sta;	//在exit.c里 面定义
extern OS_EVENT * Sem_Task_HCSR04_START;
extern OS_EVENT * Sem_Task_HCSR04_STOP;
extern uint16_t hcsr04_time;
#define IWDOG_ENABLE 0
#define NRF24L01_ENABLE 0
#define HMC5883_ENABLE 0
int main(void)
{
#if SUPPORT_OV7670==1
	u8 lightmode=0,saturation=2,brightness=2,contrast=2;
	u8 effect=0;	
#endif
	
	delay_init(168);       //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组配置
#if EN_USART1_RX 
	uart_init(115200);    //串口波特率设置
#endif
	
	LED_Init();  //LED初始化 PA6 PA7对应LED
	PWM_Capture_Init(0xFFFF,83);  //1MHZ计数 TIM4 TIM5 用来捕捉遥控器8通道输入
	PWM_MOTO_Config(); //TIM3 用来驱动 PWM四路电机
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
//	OpticalFlow_init();
	MPU6050_Init();
#if HMC5883_ENABLE ==1
	HMC5883_Init();
#endif
#if NRF24L01_ENABLE == 1
	NRF24L01_Init();
#endif
	Control_Init();
//	HCSR04_Init();
	TIM7_Int_Init(0xFFFF,83);//1Mhz的计数频率,1us时间度量	
#if IWDOG_ENABLE == 1
  IWDG_Init(4,500);//与分频数为64,重载值为500,溢出时间为1s	
#endif 
 GPIO_ResetBits(GPIOA,GPIO_Pin_6 );//灯亮表示程序初始化没问题
	
	OSInit();  //UCOS初始化
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO); //创建开始任务
	OSStart(); //开始任务
}

//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
	Sem_Task_HCSR04_START    = OSSemCreate(0);
	Sem_Task_HCSR04_STOP    = OSSemCreate(0);	
	OSStatInit();  //开启统计任务
	
	OS_ENTER_CRITICAL();  //进入临界区(关闭中断)
	OSTaskCreate(mpu6050_task,(void*)0,(OS_STK*)&MPU6050_TASK_STK[MPU6050_STK_SIZE-1],MPU6050_TASK_PRIO);//创建MPU6050任务
#if HMC5883_ENABLE ==1
	OSTaskCreate(hmc5883_task,(void*)0,(OS_STK*)&HMC5883_TASK_STK[HMC5883_STK_SIZE-1],HMC5883_TASK_PRIO);//创建HMC5883任务
#endif

	OSTaskCreate(led0_task,(void*)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);//创建LED0任务
//	OSTaskCreate(led1_task,(void*)0,(OS_STK*)&LED1_TASK_STK[LED1_STK_SIZE-1],LED1_TASK_PRIO);//创建LED1任务
//	OSTaskCreate(float_task,(void*)0,(OS_STK*)&FLOAT_TASK_STK[FLOAT_STK_SIZE-1],FLOAT_TASK_PRIO);//创建浮点测试任务
//	OSTaskCreate(hcsr04_task,(void*)0,(OS_STK*)&HCSR04_TASK_STK[HCSR04_STK_SIZE-1],HCSR04_TASK_PRIO);//创建HCSR04任务
//	OSTaskCreate(OpticalFlow_task,(void*)0,(OS_STK*)&OpticalFlow_TASK_STK[OpticalFlow_STK_SIZE-1],OpticalFlow_TASK_PRIO);//创建WFT07任务
	
	OSTaskCreate(wft_task,(void*)0,(OS_STK*)&WFT_TASK_STK[WFT_STK_SIZE-1],WFT_TASK_PRIO);//创建WFT07任务
#if NRF24L01_ENABLE ==1
	OSTaskCreate(nrf_task,(void*)0,(OS_STK*)&NRF_TASK_STK[NRF_STK_SIZE-1],NRF_TASK_PRIO);//创建NRF任务
#endif
	
#if SUPPORT_OV7670==1
		OSTaskCreate(ov7670_task,(void*)0,(OS_STK*)&OV7670_TASK_STK[OV7670_STK_SIZE-1],OV7670_TASK_PRIO);//创建NRF任务
#endif
//	OSTaskCreate(sysinfo_task,(void*)0,(OS_STK*)&SYSINFO_TASK_STK[SYSINFO_STK_SIZE-1],SYSINFO_TASK_PRIO);//创建NRF任务
	OSTaskSuspend(START_TASK_PRIO);//挂起开始任务
	OS_EXIT_CRITICAL();  //退出临界区(开中断)
}
 

//mpu6050
void mpu6050_task(void *pdata)
{
	
//	OS_CPU_SR cpu_sr=0;
	u8 ms1=0;
//	u8 time1,time,time2;
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
#if IWDOG_ENABLE==1
				IWDG_Feed();//喂狗
#endif
				ms1=0;
			}
			delay_ms(3);
//time1 = OSTimeGet();
		
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

//OpticalFlow_task
void OpticalFlow_task(void *pdata)
{
	static u8 OpticalFlow_frame[22];
		float IMU_SPEED_Z_tmp,IMU_SPEED_Z_sum,flow_comp_m_x,flow_comp_m_y;
	static uint32_t last_time=0, now_time=0; // 采样周期计数 单位 us
	static float speed_Z[5],last_HCSR04_Distance=0;	
	u8 i,ms=0;

	while(1)
	{
		GetData_OpticalFlow(0x0, OpticalFlow_frame);
		

		if(ms==0 || ms==1)
		{
			flow_comp_m_x = OpticalFlow_frame[6]<<8 | OpticalFlow_frame[7];
			flow_comp_m_y = OpticalFlow_frame[8]<<8 | OpticalFlow_frame[9];
			ms++;
		}
		if(ms==2)
		{
			HCSR04_Distance = OpticalFlow_frame[20]<<8 |OpticalFlow_frame[21];

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
			last_HCSR04_Distance = HCSR04_Distance;
			
			HCSR04_Update = 1;
			ms=0;
		}
		delay_ms(15);
	}
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
//		UserData[0]=OSCPUUsage;
	Send_Data();

	


//		Data_Send_UserData();

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
//LED0任务
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

//LED1任务
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

//浮点测试任务
void float_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	static float float_num=0.01;
	while(1)
	{
		float_num+=0.01f;
		OS_ENTER_CRITICAL();	//开中断
		printf("float_num的值为: %f\r\n",float_num); //串口打印结果
		OS_EXIT_CRITICAL();		//关中断
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

		OV7670_RRST=0;				//开始复位读指针 
		OV7670_RCK=0;
		OV7670_RCK=1;
		OV7670_RCK=0;
		OV7670_RRST=1;				//复位读指针结束 
		OV7670_RCK=1;  
		for(j=0;j<76800;j++)
		{
			OV7670_RCK=0;
			color=GPIOD->IDR&0XFF;	//读数据
			OV7670_RCK=1; 
			color<<=8;  
			OV7670_RCK=0;
			color|=GPIOD->IDR&0XFF;	//读数据
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
//		EXTI->PR=1<<9;     			//清除LINE9上的中断标志位
		ov_sta=0;					//开始下一次采集
		//	ov_frame++; 
	} 
}	
