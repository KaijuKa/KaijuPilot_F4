#include "delay.h"

////////////////////////////////////////////////////////////////////////////////// 	 

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
u32 systime_ms = 0;                   //当前系统时间
	   
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;				//为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
}								    
		    								   
void delay_us(u32 nus)
{		
	/*
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 //清空计数器	 
	*/
	u32 now = GetSysTime_us();
  while ( GetSysTime_us() - now < nus );
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(u16 nms)
{	
	/*
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	 
	*/
	u32 now = systime_ms;
  while ( systime_ms - now < nms );
} 

/*******************************************************************************
* 函 数 名         : SysTick_IT_Init
* 函数功能		     : 系统时钟1ms中断 应放于所有delay后
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SysTick_IT_Init(void)
{
	fac_us=SystemCoreClock/8000000;				                //为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					                    //非OS下,代表每个ms需要的systick时钟数   
	SysTick_Config(fac_ms);                               //节拍 1ms一次
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
}

void SysTick_Handler(void)
{
	systime_ms++;
}

//返回当前系统时间ms
u32 GetSysTime_ms(void)
{
	return systime_ms;
}

//返回当前系统时间us
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


































