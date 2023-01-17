#ifndef __POS_CALCU_H__
#define __POS_CALCU_H__

#include "stm32f4xx.h"

typedef struct{
	float baro_height;        //cm
	float baro_height_offset;
	float fusion_height;
	float baro_velD;       //气压计得到的速度cm/s
	
	u8 fix_sta;            //定位状态
	u8 star_num;           //卫星数
	float log;             //deg  经度
	float lat;             //deg  维度
	float hMSL;            //cm   高于平均水平面高度
	float hAcc;            //cm   水平精度估计
	float cAcc;            //cm   垂直精度估计
	float velN;            //cm/s 向北速度
	float velE;            //cm/s 向东速度
	float gSpeed;          //cm/s 地速
	float spdAcc;          //cm/s 速度精度估计
	float headMot;         //deg  2D运动方向
	float headAcc;         //deg  运动方向精度估计
	float pDop;            //位置精度因子 在飞控上该值小于4表示良好
} POS_Structure;

extern POS_Structure pos_data;

void POS_Init(void);
void RAW_Height_Calibration(void);
void POS_Update(u8 dT_ms);
void RAW_Height_Update(u8 dT_ms);
void Height_Fusion(u8 dT_ms);
#endif
