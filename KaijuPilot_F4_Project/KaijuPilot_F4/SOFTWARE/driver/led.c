#include "led.h"

/*******************************************************************************
* 函 数 名         : DRV_LED_Init
* 函数功能		   : 初始化led
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
}

/*******************************************************************************
* 函 数 名         : DRV_LED_LIGHTING
* 函数功能		   : 控制led亮灭
* 输    入         : 1亮 0灭
* 输    出         : 无
*******************************************************************************/
void DRV_LED_LIGHTING(u8 enable)
{
	if(1 == enable)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
	}
	else
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
	}
}
