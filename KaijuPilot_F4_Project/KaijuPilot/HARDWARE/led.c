#include "led.h"

void LED_Init(void)
{ 
 GPIO_InitTypeDef  GPIO_InitStructure;	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能GPIOB时钟 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				      //PB0
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		  //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //设为50MHZ 
 GPIO_Init(GPIOB, &GPIO_InitStructure);                 //初始化PB0
 GPIO_SetBits(GPIOB,GPIO_Pin_0);						            //将PB0置位				 
}
