#ifndef __PPM_H__
#define __PPM_H__

#include "io.h"
#define MID_PWM 1500
extern u16 ppm_output[8];

void TIM4_CH4_PPM_Init(u32 pre,u16 psc);

#endif
