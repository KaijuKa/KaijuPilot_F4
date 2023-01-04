#ifndef __REMOTE_SIGNAL_H__
#define __REMOTE_SIGNAL_H__

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

#define OFFLINE_DEADLINE 2000
#define PULSE_MIN 800
#define PULSE_MAX 2200

#define IS_SBUS

//遥控信号解析结构体
typedef struct{
	s16 sbus_ch[16];      //sbus接收到的原始数据
	u16 ppm_ch[8];        //ppm接收到的原始数据
	u8 sbus_flag;         //sbus解析中的flag
	s16 ch_processed[16]; //处理后的信号值 -500 ~ 500
	u8 sbus_offline;      //sbus掉线标志
	u8 ppm_offline;       //ppm掉线标志
}RC_Data_structure;

enum
{
 CH_ROL = 0, //横滚
 CH_PIT ,    //俯仰
 CH_THR ,    //油门
 CH_YAW ,    //偏航
 AUX1 ,      //功能开关1
 AUX2 ,      //功能开关2
 AUX3 ,      //功能开关3
 AUX4 ,      //功能开关4
 CH_NUM,//8
};

extern RC_Data_structure rc_data;

extern QueueHandle_t rc_data_queue;

void RC_Init(void);
void RC_Offline_Check(u8 dT_ms);
void RC_Data_Share(void);

void RC_SBUS_ByteGet(u8 data);
void RC_SBUS_Analysis(void);
void RC_SBUS_Offline_Reset(void);
void RC_SBUS_Offline_Check(u8 dT_ms);
void RC_SBUS_Data_Limited(void);

void TIM1_CH1_Input_Init(u32 pre,u16 psc);
void TIM1_CC_IRQHandler(void);
void RC_PPM_Calcu(u32 Pulselength);
void RC_PPM_Watch_Dog_Feed(u8 ch_n);
void RC_PPM_Offline_Check(u8 dT_ms);
void RC_PPM_Data_Limited(void);


#endif
