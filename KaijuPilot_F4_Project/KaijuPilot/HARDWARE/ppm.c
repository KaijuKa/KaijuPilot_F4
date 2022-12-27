#include "ppm.h"

u16 ppm_output[8]={1500,1500,1000,1500,1500,1500,1500,1500};

/*******************************************************************************
* �� �� ��         : TIM4_CH4_PPM_Init
* ��������		     : TIM4_CH4_PPM��ʼ��
* ��    ��         : ����ֵpre ��Ƶϵ��psc
* ��    ��         : ��
*******************************************************************************/
void TIM4_CH4_PPM_Init(u32 pre,u16 psc)
{
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NvicInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //��ʼ���ṹ�崴����ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //��ʼ������
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ��gpio�ܽ�Ϊ����
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE); //�˿�ӳ�亯��
	
	Tim_InitStructure.TIM_Period = pre;//����ֵ
	Tim_InitStructure.TIM_Prescaler = psc;//��Ƶ
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//��������
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//��Ƶ
	TIM_TimeBaseInit(TIM4,&Tim_InitStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; //����PWMģʽ2
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High; //���ü���Ϊ��
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable; //ʹ��
	TIM_OCInitStructure.TIM_Pulse = 500; //500us�͵�ƽ
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable); //ʹ��TIMx�� CCR4 �ϵ�Ԥװ�ؼĴ���
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//���������ж�
	
	NvicInitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NvicInitStructure);//NVIC��ʼ������
	
	TIM_Cmd(TIM4,ENABLE);
}

/*******************************************************************************
* �� �� ��         : TIM4_IRQHandler
* ��������		     : ���ж�������arr �Բ������ʵ��ź�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void TIM4_IRQHandler()
{
	static u16 pulseLength;
	static u8 count;
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//����жϱ�־λ
	//һ���˸�ͨ����ֵ �����˸������500us��ʣ��Ϊ�� ��ȫ20000us
	if(count == 8)
	{
		TIM_SetAutoreload(TIM4,20000-pulseLength-1);
		pulseLength = 0;
		count = 0;
	}
	//����crr�����Ӧͨ����ֵ �̶�500us��ʣ�µĸߵ�ƽΪ500us-1500us
	else
	{
		TIM_SetAutoreload(TIM4,ppm_output[count]-1);
		pulseLength += ppm_output[count]-1;
		count++;
	}
}

