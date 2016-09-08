#include "pwm_capture.h"
//#include "wft_controller.h"
#include "includes.h"

u8  TIM5CH1_CAPTURE_STA=0,TIM4CH1_CAPTURE_STA=0;	//���벶��״̬
u8  TIM5CH2_CAPTURE_STA=0,TIM4CH2_CAPTURE_STA=0;	//���벶��״̬
u8  TIM5CH3_CAPTURE_STA=0,TIM4CH3_CAPTURE_STA=0;	//���벶��״̬
u8  TIM5CH4_CAPTURE_STA=0,TIM4CH4_CAPTURE_STA=0;	//���벶��״̬		    				
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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ��TIM4ʱ��
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //ʹ��GPIO�����AFIO���ù���ģ��ʱ��ʹ��


	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //��ʼ��PD 
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //PD����λ��ʱ��4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
	
	
	
	//��ʼ����ʱ��4 TIM4	 
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM4���벶�����
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;
  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
//	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;
//  	TIM_ICInit(TIM4, &TIM4_ICInitStructure);
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM4,TIM_IT_CC2,ENABLE);	
	TIM_ITConfig(TIM4,TIM_IT_CC3,ENABLE);
//  TIM_ITConfig(TIM4,TIM_IT_CC4,ENABLE);	
	
  TIM_Cmd(TIM4,ENABLE ); 	//ʹ�ܶ�ʱ��4
}


void TIM5_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM5_ICInitStructure;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //GPIOA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //PA0����λ��ʱ��5
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM5);

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	

	//��ʼ��TIM5���벶�����
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM4, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM_ICInit(TIM5, &TIM5_ICInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	TIM_ITConfig(TIM5,TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM5,TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM5,TIM_IT_CC3,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM5,TIM_IT_CC4,ENABLE);//��������ж� ,����CC1IE�����ж�	
	
  TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��5

}
//��ʱ��4�жϷ������
void TIM4_IRQHandler(void)
{ 
	u16 temp;
	OSIntEnter();   
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)//����1���������¼�
	{	
		if(TIM4CH1_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[8]=TIM_GetCapture1(TIM4);
			TIM4CH1_CAPTURE_STA=1;		
	   	TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
		}
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);		    
	}
	
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)//����2���������¼�
	{	
		if(TIM4CH2_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC2P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[10]=TIM_GetCapture2(TIM4);
			TIM4CH2_CAPTURE_STA=1;		
	   		TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC2P=1 ����Ϊ�½��ز���
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC2);		    
	}

	if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)//����3���������¼�
	{	
		if(TIM4CH3_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC3P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[12]=TIM_GetCapture3(TIM4);
			TIM4CH3_CAPTURE_STA=1;		
	   		TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC3P=1 ����Ϊ�½��ز���
		}
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC3);		    
	}
	
//	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)//����4���������¼�
//	{	
//		if(TIM4CH4_CAPTURE_STA==1)		//����һ���½��� 		
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
//	   		TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising); //CC4P=0 ����Ϊ�����ز���
//		}else  						//��ǲ�����������
//		{
// 			Pwm_Room[14]=TIM_GetCapture4(TIM4);
//			TIM4CH4_CAPTURE_STA=1;		
//	   		TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);		//CC4P=1 ����Ϊ�½��ز���
//		}
//		TIM_ClearITPendingBit(TIM4,TIM_IT_CC4);		    
//	}			     	    					   
	//TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //����жϱ�־λ 
 	OSIntExit();

}

//��ʱ��5�жϷ������


void TIM5_IRQHandler(void)
{ 
	u16 temp;
	OSIntEnter(); 	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//����1���������¼�
	{	
		if(TIM5CH1_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[0]=TIM_GetCapture1(TIM5);
			TIM5CH1_CAPTURE_STA=1;		
	   		TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
		}
		
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC1);		    
	}
	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//����2���������¼�
	{	
		if(TIM5CH2_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC2P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[2]=TIM_GetCapture2(TIM5);
			TIM5CH2_CAPTURE_STA=1;		
	   		TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC2P=1 ����Ϊ�½��ز���
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC2);		    
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)//����3���������¼�
	{	
		if(TIM5CH3_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC3PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC3P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[4]=TIM_GetCapture3(TIM5);
			TIM5CH3_CAPTURE_STA=1;		
	   		TIM_OC3PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC3P=1 ����Ϊ�½��ز���
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC3);		    
	}
	
	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)//����4���������¼�
	{	
		if(TIM5CH4_CAPTURE_STA==1)		//����һ���½��� 		
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
	   		TIM_OC4PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC4P=0 ����Ϊ�����ز���
		}else  						//��ǲ�����������
		{
 			Pwm_Room[6]=TIM_GetCapture4(TIM5);
			TIM5CH4_CAPTURE_STA=1;		
	   		TIM_OC4PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC4P=1 ����Ϊ�½��ز���
		}
		TIM_ClearITPendingBit(TIM5,TIM_IT_CC4);		    
	}			     	    					   
	//TIM_ClearITPendingBit(TIM5,TIM_IT_Update); //����жϱ�־λ 
  	OSIntExit();

}
