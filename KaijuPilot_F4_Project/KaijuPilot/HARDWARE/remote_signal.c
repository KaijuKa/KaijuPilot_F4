#include "remote_signal.h"
#include "delay.h"
#include "kaiju_math.h"

RC_STRUCTURE rc_in;
u16 deadline_count;

/*******************************************************************************
* �� �� ��         : Channel_data_ByteGet
* ��������		     : ͨ�������ֽڽ���
* ��    ��         : u8����
* ��    ��         : ��
*******************************************************************************/
void Channel_data_ByteGet(u8 data)
{
	const u8 frame_end[4] = {0x04, 0x14, 0x24, 0x34};
	static u32 sbus_time[2];
	static u8 datatmp[25];
	static u8 cnt = 0;
	static u8 frame_cnt;
	u8 i;
	//
	sbus_time[0] = sbus_time[1];
	sbus_time[1] = GetSysTime_us();
	if ((u32)(sbus_time[1] - sbus_time[0]) > 2500)
	{
		cnt = 0;
	}
	//
	datatmp[cnt++] = data;
	//
	if (cnt == 25)
	{
		cnt = 24;
		//
		//if(datatmp[0] == 0x0F && (datatmp[24] == 0x00))
		//if(datatmp[0] == 0x0F && ((datatmp[24] == 0x00)||(datatmp[24] == 0x04)||(datatmp[24] == 0x14)||(datatmp[24] == 0x24)||(datatmp[24] == 0x34)))
		if ((datatmp[0] == 0x0F && (datatmp[24] == 0x00 || datatmp[24] == frame_end[frame_cnt])))
		{
			cnt = 0;
			rc_in.sbus_ch[0] = (s16)(datatmp[2] & 0x07) << 8 | datatmp[1];
			rc_in.sbus_ch[1] = (s16)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
			rc_in.sbus_ch[2] = (s16)(datatmp[5] & 0x01) << 10 | ((s16)datatmp[4] << 2) | (datatmp[3] >> 6);
			rc_in.sbus_ch[3] = (s16)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
			rc_in.sbus_ch[4] = (s16)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
			rc_in.sbus_ch[5] = (s16)(datatmp[9] & 0x03) << 9 | ((s16)datatmp[8] << 1) | (datatmp[7] >> 7);
			rc_in.sbus_ch[6] = (s16)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
			rc_in.sbus_ch[7] = (s16)datatmp[11] << 3 | (datatmp[10] >> 5);

			rc_in.sbus_ch[8] = (s16)(datatmp[13] & 0x07) << 8 | datatmp[12];
			rc_in.sbus_ch[9] = (s16)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
			rc_in.sbus_ch[10] = (s16)(datatmp[16] & 0x01) << 10 | ((s16)datatmp[15] << 2) | (datatmp[14] >> 6);
			rc_in.sbus_ch[11] = (s16)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
			rc_in.sbus_ch[12] = (s16)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
			rc_in.sbus_ch[13] = (s16)(datatmp[20] & 0x03) << 9 | ((s16)datatmp[19] << 1) | (datatmp[18] >> 7);
			rc_in.sbus_ch[14] = (s16)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
			rc_in.sbus_ch[15] = (s16)datatmp[22] << 3 | (datatmp[21] >> 5);
			rc_in.sbus_flag = datatmp[23];

			
			//֡β����
			frame_cnt++;
			frame_cnt %= 4;
			//user
			//
			if (rc_in.sbus_flag & 0x08)
			{
				//������������ܽ��յ���ʧ�ر�ǣ��򲻴���ת�޳�������ʧ�ء�
				;
			}
			else
			{
				Channel_data_Analysis();
			}
		}
		//������������һ������ �����������
		else
		{
			for (i = 0; i < 24; i++)
			{
				datatmp[i] = datatmp[i + 1];
			}
		}
	}
}

/*******************************************************************************
* �� �� ��         : Channel_data_Analysis
* ��������		     : ���ݽ���
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Channel_data_Analysis(void)
{
	u8 i;
	//ͨ��ֵ����
	for(i = 0;i<8;i++)
	{
		rc_in.ch_processed[i] = 0.644f * (rc_in.sbus_ch[i] - 1000);
	}
	rc_in.sbus_offline = 0;
	Channel_offline_reset();
}

/*******************************************************************************
* �� �� ��         : Channel_offline_reset
* ��������		     : ͨ������ʱ������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Channel_offline_reset(void)
{
	deadline_count = 0;
}

/*******************************************************************************
* �� �� ��         : Channel_offline_check
* ��������		     : ͨ�����߼��
* ��    ��         : ���� ms
* ��    ��         : ��
*******************************************************************************/
void Channel_offline_check(u8 dT_ms)
{
	if(deadline_count < OFFLINE_DEADLINE)
	{
		deadline_count += dT_ms;
	}
	else
	{
		rc_in.sbus_offline = 1;
	}
}

/*******************************************************************************
* �� �� ��         : TIM1_CH1_Input_Init
* ��������		     : TIM1 ch1 ppm���� ��ʼ��
* ��    ��         : u32 pre,u16 psc
* ��    ��         : ��
*******************************************************************************/
void TIM1_CH1_Input_Init(u32 pre,u16 psc)
{
	NVIC_InitTypeDef NvicInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); //��ʼ���ṹ�崴����ʱ�ӳ�ʼ��
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPD; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//��ʼ��gpio�ܽ�Ϊ����
	
	Tim_InitStructure.TIM_Period = pre;//����ֵ
	Tim_InitStructure.TIM_Prescaler = psc;//��Ƶ
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//��������
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//��Ƶ
	TIM_TimeBaseInit(TIM1,&Tim_InitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICFilter=0x00;  //�˲����� �����ڲ��˲�
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; //��Ƶϵ��
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI; //ӳ��
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);//���������ж�
	
	NvicInitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NvicInitStructure);//NVIC��ʼ������

	TIM_Cmd(TIM1,ENABLE);
}

/*******************************************************************************
* �� �� ��         : TIM1_CC_IRQHandler
* ��������		     : TIM1�жϷ�����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
	u32 Pulselength;                    //������
	u32 end_counter;                    //���忪ͷ�ļ�ʱ��
	static u32 begin_counter = 0;       //�����β�ļ�ʱ��
	end_counter = TIM_GetCapture1(TIM1);//��ȡ����ʱ�ļ�ʱ��
	if(begin_counter > end_counter)     //��0����
	{
		Pulselength = end_counter+0xffff-begin_counter;
	}
	else
	{
		Pulselength = end_counter - begin_counter;
	}                        
	begin_counter = end_counter;        //����
	PPM_calcu(Pulselength);             //��������
																		//�����־λ
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
}

/*******************************************************************************
* �� �� ��         : PPM_calcu
* ��������		     : ppm���㺯��
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void PPM_calcu(u32 Pulselength)
{
	static u8 channel_num = 0;          //��ʶ��ǰ�����ͨ����
	
	if(Pulselength > 5000)              //����5000��ʾһ��ppm����
	{
		channel_num = 0;
	}
	else
	{                                   //���ں���ͨ��ֵ�� ���� ��������
		if(Pulselength > PULSE_MIN && Pulselength < PULSE_MAX)
		{
			if(channel_num > CH_NUM)        //����ͨ���� �˳�
				return;
			
			rc_in.ppm_ch[channel_num] = Pulselength;
			ch_watch_dog_feed(channel_num); //ι��
			channel_num++;
		}
	}
}

/*******************************************************************************
* �� �� ��         : ch_watch_dog_feed
* ��������		     : ppmι�� 
* ��    ��         : ͨ��
* ��    ��         : ��
*******************************************************************************/
u16 ch_offline_time[CH_NUM];          //ͨ��ֵ��ʧʱ��
void ch_watch_dog_feed(u8 ch_n)       //ι��
{
	ch_offline_time[ch_n] = 0;
}

/*******************************************************************************
* �� �� ��         : ch_offline_check
* ��������		     : ppm ���߼��
* ��    ��         : ����
* ��    ��         : ��
*******************************************************************************/
void ch_offline_check(u8 dT_ms)       //ͨ��ֵ��ʧ���
{
	u8 i;
	for(i = 0;i<CH_NUM;i++)
	{
		//����500ms����λ��ʧ
		if(ch_offline_time[i] > OFFLINE_DEADLINE)
		{
			rc_in.ppm_offline |= 1<<i;
		}
		else
		{
			ch_offline_time[i] += dT_ms;
			rc_in.ppm_offline &= ~(1<<i);
		}
	}
}

/*******************************************************************************
* �� �� ��         : ch_data_limited
* ��������		     : ppmͨ��ֵ����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ch_data_limited(void)
{
	u8 i;
	for(i = 0;i<CH_NUM;i++)
	{
		if(rc_in.ppm_offline&(1 << i))
		{
			if(2 == i)
			{
				rc_in.ch_processed[i] = -500;
			}
			else
			{
				rc_in.ch_processed[i] = 0;
			}
		}
		else
		{
			rc_in.ch_processed[i] = (s16)rc_in.ppm_ch[i]-1500;
			rc_in.ch_processed[i] = LIMIT(rc_in.ch_processed[i],-500,500);
		}
	}
}
