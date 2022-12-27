#include "pwm.h"
#include "kaiju_math.h"
#include "arg_manage.h"

/*******************************************************************************
* �� �� ��         : TIM4_PWM_Init
* ��������		     : TIM4_PWM���
* ��    ��         : ����ֵpre ��Ƶϵ��psc
* ��    ��         : ��
*******************************************************************************/
void TIM4_PWM_Init(u32 pre,u16 psc)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //��ʼ���ṹ�崴����ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //��ʼ������
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ��gpio�ܽ�Ϊ����
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE); //�˿�ӳ�亯��
	
	Tim_InitStructure.TIM_Period = pre;//����ֵ
	Tim_InitStructure.TIM_Prescaler = psc;//��Ƶ
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//��������
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//��Ƶ
	TIM_TimeBaseInit(TIM4,&Tim_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1; //����PWMģʽ1
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //���ü���Ϊ��
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //ʹ��
	TIM_OCInitStructure.TIM_Pulse =  1500; //��ʼΪ1500
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC2Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse =  1000; //��ʼΪ1000 ����
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIMx�� CCR3 �ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ʹ��Ԥװ�ؼĴ���

	TIM_Cmd(TIM4,ENABLE);
}

/*******************************************************************************
* �� �� ��         : pwm_output
* ��������		     : pwm���
* ��    ��         : -500 ~ 500 ���ƫ�úͷ����ڴ˴���
* ��    ��         : ��
*******************************************************************************/
void pwm_output(s16 ch1, s16 ch2, s16 ch3, s16 ch4)
{
	s16 ch1_output_triangle;
	s16 ch2_output_triangle;
	
	//����ƫ��
	ch1 += flight_arg.ch1_offset;
	ch2 += flight_arg.ch2_offset;
	ch3 += flight_arg.ch3_offset;
	ch4 += flight_arg.ch4_offset;
	
	//���淽��
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
	
	//�޷�
	ch1 = LIMIT(ch1, -500, 500);
	ch2 = LIMIT(ch2, -500, 500);
	ch3 = LIMIT(ch3, -500, 500);
	ch4 = LIMIT(ch4, -500, 500);
	
	//���������
	if(0 == flight_arg.ctrl_mode)
	{
		//������޷�
		ch1_output_triangle = LIMIT(ch2+ch1, -500, 500);
		ch2_output_triangle = LIMIT(ch2-ch1, -500, 500);
		
		TIM_SetCompare4(TIM4, (u16)(ch1_output_triangle+1500));
		TIM_SetCompare3(TIM4, (u16)(ch2_output_triangle+1500));
		TIM_SetCompare2(TIM4, (u16)(ch3+1500));
		TIM_SetCompare1(TIM4, 1500);
	}
	//��ͳ����
	else
	{
		TIM_SetCompare4(TIM4, (u16)(ch1+1500));
		TIM_SetCompare3(TIM4, (u16)(ch2+1500));
		TIM_SetCompare2(TIM4, (u16)(ch3+1500));
		TIM_SetCompare1(TIM4, (u16)(ch4+1500));
	}
}
