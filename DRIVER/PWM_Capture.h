#ifndef __PWM_CAPTURE_H
#define __PWM_CAPTURE_H
#include "stm32f4xx.h"

extern u8  TIM4CH1_CAPTURE_STA;	// ‰»Î≤∂ªÒ◊¥Ã¨		    				
extern u16 Pwm_In[8];
extern u16 Pwm_Room[16];
void TIM4_Init(u32 arr,u16 psc);
void TIM5_Init(u32 arr,u16 psc);
void PWM_Capture_Init(u32 arr,u16 psc);

#endif

