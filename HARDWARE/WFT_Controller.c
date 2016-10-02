#include "wft_controller.h"
#include "control.h"
#include "moto.h"
#include "PID.h"
#include "data_transfer.h"
//Ĭ������
u8 Lock_dataTransfer = 1;  
u8 Lock_Motor = 1;
void Pwm_In_Convert(void)
{
	//Pwm_in  1000~2000
	RCTarget.Throttle = Pwm_In[2]; 
//	THROTTLE = RCTarget.Throttle;
	if(Pwm_In[0]>1522 || Pwm_In[0]<1519)	RCTarget.Yaw = ((float)(Pwm_In[0])-1520)/25.0f; //-40~40��
	else RCTarget.Yaw = 0;

	if(Pwm_In[1]>1522 || Pwm_In[1]<1519)	RCTarget.Pitch = ((float)(Pwm_In[1])-1520)/25.0f; //-40~40��
	else RCTarget.Pitch = 0;
	
	if(Pwm_In[3]>1522 || Pwm_In[3]<1519)	RCTarget.Roll = ((float)(Pwm_In[3])-1520)/25.0f-6.0f; //-40~40��
	else RCTarget.Roll = 0;
	

}

void WFT_CheckLock(void)
{
	static u8 wft_state=0;
	
			if (Pwm_In[6]<1200 ) 
			{
				Lock_dataTransfer = 1;
			}
			else if (Pwm_In[6]<1600 ) 
			{	
				Lock_dataTransfer = 0;
				Auto_Fixed_High = 0;
			} 
			else 
				Auto_Fixed_High = 1;
				
	switch(wft_state)
	{
		case 0:	 //����״̬���ȴ�����
				Lock_Motor = 1;
				if (Pwm_In[3]>1800 && Pwm_In[1]<1100) 
				{
					wft_state=1;
					GPIO_ResetBits(GPIOA,GPIO_Pin_6 );//������ʾ�����ɹ�
				}
				break;
		case 1:	  //����״̬���ȴ�����
				Lock_Motor = 0;
				if (Pwm_In[3]>1800 && Pwm_In[1]>1900) 
				{	
					wft_state=0;
					GPIO_SetBits(GPIOA,GPIO_Pin_6 );//�����ʾ����״̬
				}
				break;										
		default:
				//wft_state=0;
				break;
	}			 
				
}
