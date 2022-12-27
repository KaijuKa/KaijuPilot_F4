#ifndef __PWM_H__

#define __PWM_H__

#include "stm32f4xx.h"

void DRV_PWM_Init(void);
void DRV_PWM_Output(s16 ch1, s16 ch2, s16 ch3, s16 ch4);

#endif
