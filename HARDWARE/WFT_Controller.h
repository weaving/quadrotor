#ifndef __WFT_CONTROLLER_H
#define __WFT_CONTROLLER__H
#include "sys.h"
#include "pwm_capture.h"

extern u8 Lock_Flag;
void Pwm_In_Convert(void);
void WFT_CheckLock(void);
#endif
