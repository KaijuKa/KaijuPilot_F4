#ifndef __DELAY_H__
#define __DELAY_H__ 			   
#include "io.h"  

	 
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void SysTick_IT_Init(void);
u32 GetSysTime_ms(void);
u32 GetSysTime_us(void);
void SysTick_Handler(void);

#endif





























