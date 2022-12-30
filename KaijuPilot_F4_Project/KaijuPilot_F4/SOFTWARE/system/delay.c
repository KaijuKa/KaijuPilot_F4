#include "delay.h"
 

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
u32 systime_ms = 0;                   //当前系统时间
	   
u8 RTOS_en = 0;
	   
/*******************************************************************************
* 函 数 名         : delay_init
* 函数功能		     : 延迟初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void delay_init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;				//为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
}								    
	
/*******************************************************************************
* 函 数 名         : delay_us
* 函数功能		     : us为单位延迟
* 输    入         : us数
* 输    出         : 无
*******************************************************************************/
void delay_us(u32 nus)
{		
	u32 now = getsystime_us();
	while ( getsystime_us() - now < nus );
}

/*******************************************************************************
* 函 数 名         : delay_ms
* 函数功能		     : ms为单位延迟
* 输    入         : ms数
* 输    出         : 无
*******************************************************************************/
void delay_ms(u16 nms)
{	
	u32 now = systime_ms;
	while ( systime_ms - now < nms );
} 

/*******************************************************************************
* 函 数 名         : systick_it_init
* 函数功能		     : 系统时钟1ms中断 应放于所有delay后
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void systick_it_init(void)
{
	fac_us=SystemCoreClock/8000000;				                //为系统时钟的1/8  
	fac_ms=(u16)fac_us*1000;					                    //非OS下,代表每个ms需要的systick时钟数   
	SysTick_Config(fac_ms);                               //节拍 1ms一次
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
}

void SysTick_Handler(void)
{
	if(RTOS_en)
	{
		xPortSysTickHandler();
	}
	else
	{
		systime_ms++;
	}
}

/*******************************************************************************
* 函 数 名         : getsystime_ms
* 函数功能		     : 返回当前系统运行时间 ms单位
* 输    入         : 无
* 输    出         : ms
*******************************************************************************/
u32 getsystime_ms(void)
{
	return systime_ms;
}

/*******************************************************************************
* 函 数 名         : getsystime_us
* 函数功能		     : 返回当前系统运行时间 us单位
* 输    入         : 无
* 输    出         : us
*******************************************************************************/
u32 getsystime_us(void)
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
