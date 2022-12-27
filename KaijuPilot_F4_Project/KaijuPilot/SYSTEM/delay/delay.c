#include "delay.h"

////////////////////////////////////////////////////////////////////////////////// 	 

static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
u32 systime_ms = 0;                   //��ǰϵͳʱ��
	   
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;				//Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(u16)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
}								    
		    								   
void delay_us(u32 nus)
{		
	/*
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
	*/
	u32 now = GetSysTime_us();
  while ( GetSysTime_us() - now < nus );
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(u16 nms)
{	
	/*
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	 
	*/
	u32 now = systime_ms;
  while ( systime_ms - now < nms );
} 

/*******************************************************************************
* �� �� ��         : SysTick_IT_Init
* ��������		     : ϵͳʱ��1ms�ж� Ӧ��������delay��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SysTick_IT_Init(void)
{
	fac_us=SystemCoreClock/8000000;				                //Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(u16)fac_us*1000;					                    //��OS��,����ÿ��ms��Ҫ��systickʱ����   
	SysTick_Config(fac_ms);                               //���� 1msһ��
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
}

void SysTick_Handler(void)
{
	systime_ms++;
}

//���ص�ǰϵͳʱ��ms
u32 GetSysTime_ms(void)
{
	return systime_ms;
}

//���ص�ǰϵͳʱ��us
u32 GetSysTime_us(void)
{
  register u32 ms;
  u32 value;
	do
	{
    ms = systime_ms;
    value = ms * 1000 + ( SysTick->LOAD - SysTick->VAL ) * 1000 / SysTick->LOAD;
	}
	while(ms != systime_ms);
	return value;
}


































