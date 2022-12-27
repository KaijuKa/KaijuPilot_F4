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
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //ʹ��PORTAʱ��	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;           //PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;        //PA0���ó���������	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);               //��ʼ��GPIOA0
	
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
	static u8 key_up=1;        //�������ɿ���־
	if(mode)key_up=1;          //֧������		  
	if(key_up&&(KEY0==0))
	{
		delay_ms(10);            //ȥ���� 
		key_up=0;
		return KEY0_PRES;
	}
	else if(KEY0==1)
	{
		key_up=1;
	}
	return 0;                 // �ް�������
}
