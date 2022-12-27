#include "ppm.h"

u16 ppm_output[8]={1500,1500,1000,1500,1500,1500,1500,1500};

/*******************************************************************************
* 函 数 名         : TIM4_CH4_PPM_Init
* 函数功能		     : TIM4_CH4_PPM初始化
* 输    入         : 计数值pre 分频系数psc
* 输    出         : 无
*******************************************************************************/
void TIM4_CH4_PPM_Init(u32 pre,u16 psc)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NvicInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //初始化结构体创建及时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //初始化复用
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化gpio管脚为复用
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE); //端口映射函数
	
	Tim_InitStructure.TIM_Period = pre;//计数值
	Tim_InitStructure.TIM_Prescaler = psc;//分频
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//上升计数
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//分频
	TIM_TimeBaseInit(TIM4,&Tim_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; //设置PWM模式2
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //设置极性为高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //使能
	TIM_OCInitStructure.TIM_Pulse = 500; //500us低电平
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能TIMx在 CCR4 上的预装载寄存器
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启更新中断
	
	NvicInitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NvicInitStructure);//NVIC初始化设置
	
	TIM_Cmd(TIM4,ENABLE);
}

/*******************************************************************************
* 函 数 名         : TIM4_IRQHandler
* 函数功能		     : 在中断中设置arr 以产生合适的信号
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void TIM4_IRQHandler()
{
	static u16 pulseLength;
	static u8 count;
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//清楚中断标志位
	//一共八个通道的值 输出完八个后输出500us低剩下为高 补全20000us
	if(count == 8)
	{
		TIM_SetAutoreload(TIM4,20000-pulseLength-1);
		pulseLength = 0;
		count = 0;
	}
	//控制crr输出对应通道的值 固定500us低剩下的高电平为500us-1500us
	else
	{
		TIM_SetAutoreload(TIM4,ppm_output[count]-1);
		pulseLength += ppm_output[count]-1;
		count++;
	}
}

