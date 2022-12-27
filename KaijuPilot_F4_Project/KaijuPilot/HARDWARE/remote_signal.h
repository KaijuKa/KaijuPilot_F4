#ifndef __REMOTE_SIGNAL_H__
#define __REMOTE_SIGNAL_H__

#include "io.h"

#define OFFLINE_DEADLINE 500
#define PULSE_MIN 800
#define PULSE_MAX 2200

typedef struct{
	s16 sbus_ch[16];      //sbus接收到的原始数据
	u16 ppm_ch[8];        //ppm接收到的原始数据
	u8 sbus_flag;         //sbus解析中的flag
	s16 ch_processed[16]; //处理后的信号值 -500 ~ 500
	u8 sbus_offline;      //sbus掉线标志
	u8 ppm_offline;       //ppm掉线标志
}RC_STRUCTURE;

enum
{
 CH_ROL = 0,
 CH_PIT ,
 CH_THR ,
 CH_YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
 CH_NUM,//8
};

extern RC_STRUCTURE rc_in;

void Channel_data_ByteGet(u8 data);
void Channel_data_Analysis(void);
void Channel_offline_reset(void);
void Channel_offline_check(u8 dT_ms);

void TIM1_CH1_Input_Init(u32 pre,u16 psc);
void TIM1_CC_IRQHandler(void);
void PPM_calcu(u32 Pulselength);
void ch_watch_dog_feed(u8 ch_n);
void ch_offline_check(u8 dT_ms);
void ch_data_limited(void);


#endif
