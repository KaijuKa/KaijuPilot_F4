#include "pwm.h"
#include "kaiju_math.h"
#include "par_manage.h"

/*******************************************************************************
* 函 数 名         : DRV_PWM_Init
* 函数功能		     : PWM输出
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_PWM_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//reset st
	GPIO_StructInit(&GPIO_InitStructure);
    TIM_TimeBaseStructInit ( &TIM_TimeBaseStructure );
    TIM_OCStructInit ( &TIM_OCInitStructure );
	
	//enable clock
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//set gpio
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOB, &GPIO_InitStructure );

	//set afconfig
	GPIO_PinAFConfig ( GPIOB, GPIO_PinSource4, GPIO_AF_TIM3 );
	GPIO_PinAFConfig ( GPIOB, GPIO_PinSource6, GPIO_AF_TIM4 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource7, GPIO_AF_TIM4 );
	GPIO_PinAFConfig ( GPIOB, GPIO_PinSource8, GPIO_AF_TIM4 );
    GPIO_PinAFConfig ( GPIOB, GPIO_PinSource9, GPIO_AF_TIM4 );
	
	//set TIM3
	TIM_TimeBaseStructure.TIM_Period = 20000;
    TIM_TimeBaseStructure.TIM_Prescaler = 83;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit ( TIM3, &TIM_TimeBaseStructure );
	
	//set TIM4
	TIM_TimeBaseStructure.TIM_Period = 20000;
    TIM_TimeBaseStructure.TIM_Prescaler = 83;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit ( TIM4, &TIM_TimeBaseStructure );
	
	//set TIM4 PWM OC
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
    TIM_OC1Init ( TIM4, &TIM_OCInitStructure );
    TIM_OC1PreloadConfig ( TIM4, TIM_OCPreload_Enable );
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC2Init ( TIM4, &TIM_OCInitStructure );
    TIM_OC2PreloadConfig ( TIM4, TIM_OCPreload_Enable );
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC3Init ( TIM4, &TIM_OCInitStructure );
    TIM_OC3PreloadConfig ( TIM4, TIM_OCPreload_Enable );
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC4Init ( TIM4, &TIM_OCInitStructure );
    TIM_OC4PreloadConfig ( TIM4, TIM_OCPreload_Enable );
	
	//set TIM3 PWM OC
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500;
	TIM_OC1Init ( TIM3, &TIM_OCInitStructure );
    TIM_OC1PreloadConfig ( TIM3, TIM_OCPreload_Enable );
	
	//enable TIM4
	TIM_ARRPreloadConfig ( TIM4, ENABLE );
    TIM_Cmd ( TIM4, ENABLE );
	
	//enable TIM3
	TIM_ARRPreloadConfig ( TIM3, ENABLE );
    TIM_Cmd ( TIM3, ENABLE );
}

/*******************************************************************************
* 函 数 名         : DRV_PWM_Output
* 函数功能		     : pwm输出
* 输    入         : -500 ~ 500 输出偏置和方向都在此处理
* 输    出         : 无
*******************************************************************************/
void DRV_PWM_Output(s16 ch1, s16 ch2, s16 ch3, s16 ch4)
{
	s16 ch1_output_triangle;
	s16 ch2_output_triangle;
	
	//舵面偏置
	ch1 += fl_par.par.ch1_offset;
	ch2 += fl_par.par.ch2_offset;
	ch3 += fl_par.par.ch3_offset;
	ch4 += fl_par.par.ch4_offset;
	
	//舵面方向
	if(0 != fl_par.par.ch1_direct)
	{
		ch1 = -ch1;
	}
	if(0 != fl_par.par.ch2_direct)
	{
		ch2 = -ch2;
	}
	if(0 != fl_par.par.ch4_direct)
	{
		ch4 = -ch4;
	}
	
	//限幅
	ch1 = LIMIT(ch1, -500, 500);
	ch2 = LIMIT(ch2, -500, 500);
	ch3 = LIMIT(ch3, -500, 500);
	ch4 = LIMIT(ch4, -500, 500);
	
	//三角翼输出
	if(0 == fl_par.par.ctrl_mode)
	{
		//混合再限幅
		ch1_output_triangle = LIMIT(ch2+ch1, -500, 500);
		ch2_output_triangle = LIMIT(ch2-ch1, -500, 500);
		
		TIM_SetCompare4(TIM4, (u16)(ch1_output_triangle+1500));
		TIM_SetCompare3(TIM4, (u16)(ch2_output_triangle+1500));
		TIM_SetCompare2(TIM4, (u16)(ch3+1500));
		TIM_SetCompare1(TIM4, 1500);
	}
	//传统布局
	else
	{
		TIM_SetCompare4(TIM4, (u16)(ch1+1500));
		TIM_SetCompare3(TIM4, (u16)(ch1+1500));
		TIM_SetCompare2(TIM4, (u16)(ch3+1500));
		TIM_SetCompare1(TIM4, (u16)(ch2+1500));
	}
}
