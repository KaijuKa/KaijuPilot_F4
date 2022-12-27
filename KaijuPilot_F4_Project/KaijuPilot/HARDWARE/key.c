#include "key.h"

/*****************************************************************************
 * @name       :void KEY_Init(void)
 * @date       :2020-05-08 
 * @function   :Initialize KEY GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void KEY_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //使能PORTA时钟	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;           //PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;        //PA0设置成下拉输入	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);               //初始化GPIOA0
	
} 

/*****************************************************************************
 * @name       :u8 KEY_Scan(u8 mode)
 * @date       :2020-05-08 
 * @function   :Scan whether the KEY is pressed or not 
 * @parameters :mode: 0-not support to Press the key continuously
                      1-support to Press the key continuously
 * @retvalue   :the key value of the KEY which is pressed 
******************************************************************************/	
u8 KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;        //按键按松开标志
	if(mode)key_up=1;          //支持连按		  
	if(key_up&&(KEY0==0))
	{
		delay_ms(10);            //去抖动 
		key_up=0;
		return KEY0_PRES;
	}
	else if(KEY0==1)
	{
		key_up=1;
	}
	return 0;                 // 无按键按下
}
