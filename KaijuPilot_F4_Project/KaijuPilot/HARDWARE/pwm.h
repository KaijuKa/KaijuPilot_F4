#ifndef __PWM_H__

#define __PWM_H__

#include "io.h"


void TIM4_PWM_Init(u32 pre,u16 psc);
void pwm_output(s16 ch1, s16 ch2, s16 ch3, s16 ch4);

#endif
