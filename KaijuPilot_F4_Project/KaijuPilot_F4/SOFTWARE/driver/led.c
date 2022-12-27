#include "led.h"

/*******************************************************************************
* �� �� ��         : DRV_LED_Init
* ��������		   : ��ʼ��led
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : DRV_LED_LIGHTING
* ��������		   : ����led����
* ��    ��         : 1�� 0��
* ��    ��         : ��
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
