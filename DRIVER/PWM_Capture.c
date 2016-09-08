#include "pwm_capture.h"
//#include "wft_controller.h"
#include "includes.h"

u8  TIM5CH1_CAPTURE_STA=0,TIM4CH1_CAPTURE_STA=0;	//输入捕获状态
u8  TIM5CH2_CAPTURE_STA=0,TIM4CH2_CAPTURE_STA=0;	//输入捕获状态
u8  TIM5CH3_CAPTURE_STA=0,TIM4CH3_CAPTURE_STA=0;	//输入捕获状态
u8  TIM5CH4_CAPTURE_STA=0,TIM4CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u16 Pwm_In[8];
u16 Pwm_Room[16];

void PWM_Capture_Init(u32 arr,u16 psc)
{	 
	TIM4_Init(arr,psc);
	TIM5_Init(arr,psc);
}

void TIM4_Init(u32 arr,u16 psc)
{
	TIM_ICInitTypeDef  TIM4_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能TIM4时钟
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能GPIO外设和AFIO复用功能模块时钟使能


	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PD 
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //PD复用位定时器4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	
	
	
	//初始化定时器4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM4输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
//	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;
//  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);	
	TIM_ITConfig(TIM4,TIM_IT_CC3,ENABLE);
//  TIM_ITConfig(TIM4,TIM_IT_CC4,ENABLE);	
	
  TIM_Cmd(TIM4,ENABLE ); 	//使能定时器4
}


void TIM5_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM5_ICInitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //GPIOA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //PA0复用位定时器5
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	

	//初始化TIM5输入捕获参数
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM4, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM5,TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM5,TIM_IT_CC3,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM5,TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	
  TIM_Cmd(TIM5,ENABLE ); 	//使能定时器5

}
//定时器4中断服务程序
void TIM4_IRQHandler(void)
{ 
	u16 temp;
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
	{	
		if(TIM4CH1_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM4CH1_CAPTURE_STA=0;
					
			Pwm_Room[9]=TIM_GetCapture1(TIM4);
			if (Pwm_Room[9]>Pwm_Room[8])
			{
				temp = Pwm_Room[9]- Pwm_Room[8];
				if( (temp - Pwm_In[4] )<2000&&(temp - Pwm_In[4] )>-2000)
				{
					Pwm_In[4] = temp;
				}
			}else
			{
			//	Pwm_In[0] = 0xFF - Pwm_Room[0] + Pwm_Room[1];	
			}
	   		TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[8]=TIM_GetCapture1(TIM4);
			TIM4CH1_CAPTURE_STA=1;		
	   	TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
		}
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);		    
	}
	
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)//捕获2发生捕获事件
	{	
		if(TIM4CH2_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM4CH2_CAPTURE_STA=0;

			
			Pwm_Room[11]=TIM_GetCapture2(TIM4);
			if (Pwm_Room[11]>Pwm_Room[10])
			{
				temp = Pwm_Room[11]- Pwm_Room[10];
				if( (temp - Pwm_In[5] )<2000&&(temp - Pwm_In[5] )>-2000)
				{
					Pwm_In[5] = temp;
				}
			}else
			{
			//	Pwm_In[1] = 0xFF - Pwm_Room[2] + Pwm_Room[3];	
			} 	
	   		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[10]=TIM_GetCapture2(TIM4);
			TIM4CH2_CAPTURE_STA=1;		
	   		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC2);		    
	}

	if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)//捕获3发生捕获事件
	{	
		if(TIM4CH3_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM4CH3_CAPTURE_STA=0;
					
			Pwm_Room[13]=TIM_GetCapture3(TIM4);
			if (Pwm_Room[13]>Pwm_Room[12])
			{
				temp = Pwm_Room[13]- Pwm_Room[12];
				if( (temp - Pwm_In[6] )<2000&&(temp - Pwm_In[6] )>-2000)
				{
					Pwm_In[6] = temp;
				}

			}else
			{
			//	Pwm_In[2] = 0xFF - Pwm_Room[4] + Pwm_Room[5];	
			}
	   		TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC3P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[12]=TIM_GetCapture3(TIM4);
			TIM4CH3_CAPTURE_STA=1;		
	   		TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC3P=1 设置为下降沿捕获
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC3);		    
	}
	
//	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)//捕获4发生捕获事件
//	{	
//		if(TIM4CH4_CAPTURE_STA==1)		//捕获到一个下降沿 		
//		{	  			
//			TIM4CH4_CAPTURE_STA=0;
//					
//			Pwm_Room[15]=TIM_GetCapture4(TIM4);
//			if (Pwm_Room[15]>Pwm_Room[14])
//			{
//				Pwm_In[7] = Pwm_Room[15]- Pwm_Room[14];
//			}else
//			{
//			//	Pwm_In[3] = 0xFF - Pwm_Room[7] + Pwm_Room[6];	
//			}
//	   		TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC4P=0 设置为上升沿捕获
//		}else  						//标记捕获到了上升沿
//		{
// 			Pwm_Room[14]=TIM_GetCapture4(TIM4);
//			TIM4CH4_CAPTURE_STA=1;		
//	   		TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
//		}
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);		    
//	}			     	    					   
	//TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除中断标志位 
 	OSIntExit();

}

//定时器5中断服务程序


void TIM5_IRQHandler(void)
{ 
	u16 temp;
	OSIntEnter(); 	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
	{	
		if(TIM5CH1_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM5CH1_CAPTURE_STA=0;
					
			Pwm_Room[1]=TIM_GetCapture1(TIM5);
			if (Pwm_Room[1]>Pwm_Room[0])
			{
				temp = Pwm_Room[1]- Pwm_Room[0];
				if( (temp - Pwm_In[0] )<2000&&(temp - Pwm_In[0] )>-2000)
				{
					Pwm_In[0] = temp;
				}
				
			}else
			{
			//	Pwm_In[0] = 0xFF - Pwm_Room[0] + Pwm_Room[1];	
			}
	   		TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[0]=TIM_GetCapture1(TIM5);
			TIM5CH1_CAPTURE_STA=1;		
	   		TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
		}
		
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);		    
	}
	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//捕获2发生捕获事件
	{	
		if(TIM5CH2_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM5CH2_CAPTURE_STA=0;
					
			Pwm_Room[3]=TIM_GetCapture2(TIM5);
			if (Pwm_Room[3]>Pwm_Room[2])
			{
				temp = Pwm_Room[3]- Pwm_Room[2];
				if( (temp - Pwm_In[1] )<2000&&(temp - Pwm_In[1] )>-2000)
				{
					Pwm_In[1] = temp;
				}
				
			}else
			{
			//	Pwm_In[1] = 0xFF - Pwm_Room[2] + Pwm_Room[3];	
			} 	
	   		TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[2]=TIM_GetCapture2(TIM5);
			TIM5CH2_CAPTURE_STA=1;		
	   		TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC2);		    
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)//捕获3发生捕获事件
	{	
		if(TIM5CH3_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM5CH3_CAPTURE_STA=0;
					
			Pwm_Room[5]=TIM_GetCapture3(TIM5);
			if (Pwm_Room[5]>Pwm_Room[4])
			{
				temp = Pwm_Room[5]- Pwm_Room[4];
				if( (temp - Pwm_In[2] )<2000&&(temp - Pwm_In[2] )>-2000)
				{
					Pwm_In[2] = temp;
				}
				
			}else
			{
			//	Pwm_In[2] = 0xFF - Pwm_Room[4] + Pwm_Room[5];	
			}
	   		TIM_OC3PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC3P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[4]=TIM_GetCapture3(TIM5);
			TIM5CH3_CAPTURE_STA=1;		
	   		TIM_OC3PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC3P=1 设置为下降沿捕获
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC3);		    
	}
	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)//捕获4发生捕获事件
	{	
		if(TIM5CH4_CAPTURE_STA==1)		//捕获到一个下降沿 		
		{	  			
			TIM5CH4_CAPTURE_STA=0;
					
			Pwm_Room[7]=TIM_GetCapture4(TIM5);
			if (Pwm_Room[7]>Pwm_Room[6])
			{
				temp = Pwm_Room[7]- Pwm_Room[6];
				if( (temp - Pwm_In[3] )<2000&&(temp - Pwm_In[3] )>-2000)
				{
					Pwm_In[3] = temp;
				}
			}else
			{
			//	Pwm_In[3] = 0xFF - Pwm_Room[7] + Pwm_Room[6];	
			}
	   		TIM_OC4PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC4P=0 设置为上升沿捕获
		}else  						//标记捕获到了上升沿
		{
 			Pwm_Room[6]=TIM_GetCapture4(TIM5);
			TIM5CH4_CAPTURE_STA=1;		
	   		TIM_OC4PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC4);		    
	}			     	    					   
	//TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //清除中断标志位 
  	OSIntExit();

}
