#include "pwm.h"
#include "kaiju_math.h"
#include "arg_manage.h"

/*******************************************************************************
* 函 数 名         : TIM4_PWM_Init
* 函数功能		     : TIM4_PWM输出
* 输    入         : 计数值pre 分频系数psc
* 输    出         : 无
*******************************************************************************/
void TIM4_PWM_Init(u32 pre,u16 psc)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //初始化结构体创建及时钟初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //初始化复用
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化gpio管脚为复用
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE); //端口映射函数
	
	Tim_InitStructure.TIM_Period = pre;//计数值
	Tim_InitStructure.TIM_Prescaler = psc;//分频
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//上升计数
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//分频
	TIM_TimeBaseInit(TIM4,&Tim_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //设置PWM模式1
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //设置极性为高
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //使能
	TIM_OCInitStructure.TIM_Pulse =  1500; //初始为1500
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse =  1000; //初始为1000 油门
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能TIMx在 CCR3 上的预装载寄存器
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能TIMx在 CCR3 上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能TIMx在 CCR3 上的预装载寄存器
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); //使能TIMx在 CCR3 上的预装载寄存器
	TIM_ARRPreloadConfig(TIM4,ENABLE);//使能预装载寄存器

	TIM_Cmd(TIM4,ENABLE);
}

/*******************************************************************************
* 函 数 名         : pwm_output
* 函数功能		     : pwm输出
* 输    入         : -500 ~ 500 输出偏置和方向都在此处理
* 输    出         : 无
*******************************************************************************/
void pwm_output(s16 ch1, s16 ch2, s16 ch3, s16 ch4)
{
	s16 ch1_output_triangle;
	s16 ch2_output_triangle;
	
	//舵面偏置
	ch1 += flight_arg.ch1_offset;
	ch2 += flight_arg.ch2_offset;
	ch3 += flight_arg.ch3_offset;
	ch4 += flight_arg.ch4_offset;
	
	//舵面方向
	if(0 != flight_arg.ch1_direct)
	{
		ch1 = -ch1;
	}
	if(0 != flight_arg.ch2_direct)
	{
		ch2 = -ch2;
	}
	if(0 != flight_arg.ch4_direct)
	{
		ch4 = -ch4;
	}
	
	//限幅
	ch1 = LIMIT(ch1, -500, 500);
	ch2 = LIMIT(ch2, -500, 500);
	ch3 = LIMIT(ch3, -500, 500);
	ch4 = LIMIT(ch4, -500, 500);
	
	//三角翼输出
	if(0 == flight_arg.ctrl_mode)
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
		TIM_SetCompare3(TIM4, (u16)(ch2+1500));
		TIM_SetCompare2(TIM4, (u16)(ch3+1500));
		TIM_SetCompare1(TIM4, (u16)(ch4+1500));
	}
}
