#ifndef __DELAY_H__
#define __DELAY_H__ 

#include "stm32f4xx.h" 

extern u8 RTOS_en;
	 
void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void systick_it_init(void);
u32 getsystime_us(void);
u32 getsystime_ms(void);

#endif





























