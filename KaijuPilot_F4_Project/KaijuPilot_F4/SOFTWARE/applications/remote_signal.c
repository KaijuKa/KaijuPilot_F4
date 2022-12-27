#include "remote_signal.h"
#include "delay.h"
#include "kaiju_math.h"
#include "uart.h"

RC_Data_structure rc_data;
u16 deadline_count;

/*******************************************************************************
* 函 数 名         : RC_Init
* 函数功能		     : RC接收初始化 初始化sbus或者ppm
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void RC_Init(void)
{
#ifdef IS_SBUS
	DRV_UART5_Init(100000);
#else
	TIM1_CH1_Input_Init(0xffff-1, 72-1);
#endif
}

/*******************************************************************************
* 函 数 名         : RC_Offline_Check
* 函数功能		     : rc掉线检测 sbus或者ppm
* 输    入         : ms周期
* 输    出         : 无
*******************************************************************************/
void RC_Offline_Check(u8 dT_ms)
{
#ifdef IS_SBUS
	RC_SBUS_Offline_Check(dT_ms);
	RC_SBUS_Data_Limited();
#else
	RC_PPM_Offline_Check(dT_ms);
	RC_PPM_Data_Limited();
#endif
}

/*******************************************************************************
* 函 数 名         : RC_SBUS_ByteGet
* 函数功能		     : 通道数据字节接收
* 输    入         : u8数据
* 输    出         : 无
*******************************************************************************/
void RC_SBUS_ByteGet(u8 data)
{
	const u8 frame_end[4] = {0x04, 0x14, 0x24, 0x34};
	static u32 sbus_time[2];
	static u8 datatmp[25];
	static u8 cnt = 0;
	static u8 frame_cnt;
	u8 i;
	//
//	sbus_time[0] = sbus_time[1];
//	sbus_time[1] = GetSysTime_us();
//	if ((u32)(sbus_time[1] - sbus_time[0]) > 2500)
//	{
//		cnt = 0;
//	}
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
			rc_data.sbus_ch[0] = (s16)(datatmp[2] & 0x07) << 8 | datatmp[1];
			rc_data.sbus_ch[1] = (s16)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
			rc_data.sbus_ch[2] = (s16)(datatmp[5] & 0x01) << 10 | ((s16)datatmp[4] << 2) | (datatmp[3] >> 6);
			rc_data.sbus_ch[3] = (s16)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
			rc_data.sbus_ch[4] = (s16)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
			rc_data.sbus_ch[5] = (s16)(datatmp[9] & 0x03) << 9 | ((s16)datatmp[8] << 1) | (datatmp[7] >> 7);
			rc_data.sbus_ch[6] = (s16)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
			rc_data.sbus_ch[7] = (s16)datatmp[11] << 3 | (datatmp[10] >> 5);

			rc_data.sbus_ch[8] = (s16)(datatmp[13] & 0x07) << 8 | datatmp[12];
			rc_data.sbus_ch[9] = (s16)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
			rc_data.sbus_ch[10] = (s16)(datatmp[16] & 0x01) << 10 | ((s16)datatmp[15] << 2) | (datatmp[14] >> 6);
			rc_data.sbus_ch[11] = (s16)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
			rc_data.sbus_ch[12] = (s16)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
			rc_data.sbus_ch[13] = (s16)(datatmp[20] & 0x03) << 9 | ((s16)datatmp[19] << 1) | (datatmp[18] >> 7);
			rc_data.sbus_ch[14] = (s16)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
			rc_data.sbus_ch[15] = (s16)datatmp[22] << 3 | (datatmp[21] >> 5);
			rc_data.sbus_flag = datatmp[23];

			
			//帧尾处理
			frame_cnt++;
			frame_cnt %= 4;
			//user
			//
			if (rc_data.sbus_flag & 0x08)
			{
				//如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
				;
			}
			else
			{
				RC_SBUS_Analysis();
			}
		}
		//有问题抛弃第一个数据 接着往后接收
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
* 函 数 名         : RC_SBUS_Analysis
* 函数功能		     : 数据解析
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void RC_SBUS_Analysis(void)
{
	u8 i;
	//通道值计算
	for(i = 0;i<8;i++)
	{
		rc_data.ch_processed[i] = 0.644f * (rc_data.sbus_ch[i] - 1000);
		
		//油门缩放
		if(i == 2)
		{
			rc_data.ch_processed[i] = (s16)((rc_data.ch_processed[i]+500)*0.6f)-500;
		}
		
		rc_data.ch_processed[i] = LIMIT(rc_data.ch_processed[i], -500, 500);
	}
	rc_data.sbus_offline = 0;
	RC_SBUS_Offline_Reset();
}

/*******************************************************************************
* 函 数 名         : RC_SBUS_Offline_Reset
* 函数功能		     : 通道掉线时间清零
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void RC_SBUS_Offline_Reset(void)
{
	deadline_count = 0;
}

/*******************************************************************************
* 函 数 名         : RC_SBUS_Offline_Check
* 函数功能		     : 通道掉线检测
* 输    入         : 周期 ms
* 输    出         : 无
*******************************************************************************/
void RC_SBUS_Offline_Check(u8 dT_ms)
{
	if(deadline_count < OFFLINE_DEADLINE)
	{
		deadline_count += dT_ms;
	}
	else
	{
		rc_data.sbus_offline = 1;
	}
}

/*******************************************************************************
* 函 数 名         : RC_SBUS_Data_Limited
* 函数功能		     : SBUS掉线数据处理
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void RC_SBUS_Data_Limited(void)
{
	u8 i;
	if(rc_data.sbus_offline > 0)
	{
		for(i = 0;i<CH_NUM;i++)
		{
			if(2 == i)
			{
				rc_data.ch_processed[i] = -500;
			}
			else
			{
				rc_data.ch_processed[i] = 0;
			}
		}
	}
}

/*******************************************************************************
* 函 数 名         : TIM1_CH1_Input_Init
* 函数功能		     : TIM1 ch1 ppm输入 初始化
* 输    入         : u32 pre,u16 psc
* 输    出         : 无
*******************************************************************************/
void TIM1_CH1_Input_Init(u32 pre,u16 psc)
{
	NVIC_InitTypeDef NvicInitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef Tim_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //初始化结构体创建及时钟初始化
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化gpio管脚为复用
	
	Tim_InitStructure.TIM_Period = pre;//计数值
	Tim_InitStructure.TIM_Prescaler = psc;//分频
	Tim_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;//上升计数
	Tim_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//分频
	TIM_TimeBaseInit(TIM1,&Tim_InitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1; 
	TIM_ICInitStructure.TIM_ICFilter=0x00;  //滤波长度 现在在不滤波
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; //分频系数
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI; //映射
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	TIM_ITConfig(TIM1,TIM_IT_CC1,ENABLE);//开启捕获中断
	
	NvicInitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
	NvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NvicInitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NvicInitStructure);//NVIC初始化设置

	TIM_Cmd(TIM1,ENABLE);
}

/*******************************************************************************
* 函 数 名         : TIM1_CC_IRQHandler
* 函数功能		     : TIM1中断服务函数
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{
	u32 Pulselength;                    //脉宽长度
	u32 end_counter;                    //脉冲开头的计时数
	static u32 begin_counter = 0;       //脉冲结尾的计时数
	end_counter = TIM_GetCapture1(TIM1);//获取捕获时的计时数
	if(begin_counter > end_counter)     //过0处理
	{
		Pulselength = end_counter+0xffff-begin_counter;
	}
	else
	{
		Pulselength = end_counter - begin_counter;
	}                        
	begin_counter = end_counter;        //更新
	RC_PPM_Calcu(Pulselength);             //处理脉宽
																		//清除标志位
	TIM_ClearITPendingBit(TIM1,TIM_IT_CC1);
}

/*******************************************************************************
* 函 数 名         : RC_PPM_Calcu
* 函数功能		     : ppm计算函数
* 输    入         : 脉宽长度
* 输    出         : 无
*******************************************************************************/
void RC_PPM_Calcu(u32 Pulselength)
{
	static u8 channel_num = 0;          //标识当前处理的通道号
	
	if(Pulselength > 5000)              //大于5000表示一个ppm结束
	{
		channel_num = 0;
	}
	else
	{                                   //处于合理通道值内 处理 否则遗弃
		if(Pulselength > PULSE_MIN && Pulselength < PULSE_MAX)
		{
			if(channel_num > CH_NUM)        //大于通道数 退出
				return;
			
			rc_data.ppm_ch[channel_num] = Pulselength;
			RC_PPM_Watch_Dog_Feed(channel_num); //喂狗
			channel_num++;
		}
	}
}

/*******************************************************************************
* 函 数 名         : ch_watch_dog_feed
* 函数功能		     : ppm喂狗 
* 输    入         : 通道
* 输    出         : 无
*******************************************************************************/
u16 ch_offline_time[CH_NUM];          //通道值丢失时间
void RC_PPM_Watch_Dog_Feed(u8 ch_n)       //喂狗
{
	ch_offline_time[ch_n] = 0;
}

/*******************************************************************************
* 函 数 名         : RC_PPM_Offline_Check
* 函数功能		     : ppm 掉线检测
* 输    入         : 周期
* 输    出         : 无
*******************************************************************************/
void RC_PPM_Offline_Check(u8 dT_ms)       //通道值丢失检测
{
	u8 i;
	for(i = 0;i<CH_NUM;i++)
	{
		//大于500ms则置位丢失
		if(ch_offline_time[i] > OFFLINE_DEADLINE)
		{
			rc_data.ppm_offline |= 1<<i;
		}
		else
		{
			ch_offline_time[i] += dT_ms;
			rc_data.ppm_offline &= ~(1<<i);
		}
	}
}

/*******************************************************************************
* 函 数 名         : RC_PPM_Data_Limited
* 函数功能		     : ppm通道值限制
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void RC_PPM_Data_Limited(void)
{
	u8 i;
	for(i = 0;i<CH_NUM;i++)
	{
		if(rc_data.ppm_offline&(1 << i))
		{
			if(2 == i)
			{
				rc_data.ch_processed[i] = -500;
			}
			else
			{
				rc_data.ch_processed[i] = 0;
			}
		}
		else
		{
			rc_data.ch_processed[i] = (s16)rc_data.ppm_ch[i]-1500;
			rc_data.ch_processed[i] = LIMIT(rc_data.ch_processed[i],-500,500);
		}
	}
}
