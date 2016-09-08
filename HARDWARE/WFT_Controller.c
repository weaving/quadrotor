#include "wft_controller.h"
#include "control.h"
#include "moto.h"
#include "PID.h"
#include "data_transfer.h"
u8 Lock_Flag = 1;  //默认上锁
void Pwm_In_Convert(void)
{
	//Pwm_in  1000~2000
	RCTarget.Throttle = Pwm_In[2]; 
//	THROTTLE = RCTarget.Throttle;
	if(Pwm_In[0]>1522 || Pwm_In[0]<1519)	RCTarget.Yaw = ((float)(Pwm_In[0])-1520)/12.5f; //-40~40度
	else RCTarget.Yaw = 0;

	if(Pwm_In[1]>1522 || Pwm_In[1]<1519)	RCTarget.Pitch = ((float)(Pwm_In[1])-1520)/12.5f; //-40~40度
	else RCTarget.Pitch = 0;
	
	if(Pwm_In[3]>1522 || Pwm_In[3]<1519)	RCTarget.Roll = ((float)(Pwm_In[3])-1520)/12.5f-12; //-40~40度
	else RCTarget.Roll = 0;
	

}

void WFT_CheckLock(void)
{
				if (Pwm_In[6]<1200 ) 
				{
					Lock_Flag = 1;
				}
				else if (Pwm_In[6]<1600 ) 
				{	
					Lock_Flag = 0;
				} 
				else ;
}
