#ifndef __TECS_H__
#define __TECS_H__

#include "stm32f4xx.h"

//tecs控制结构体
typedef struct{
	float air_speed;          //当前空速 V  m/s
	float altitude;           //当前海拔高度 H m
	float track_angle;        //当前航迹角 θ 度
	float air_spd_diff;       //当前空速微分 dV/dt
	
	float target_track_angle; //目标航迹角
	float target_air_spd_diff;//目标空速微分
	
	float target_energy_diff; //目标能量微分
	float target_alloc_diff;  //目标能量分配率
	
	float energy_diff;        //当前能量微分 ((dV/dt)/g + θ)*V 巡航V基本不变 
							  //数据传递中忽略V 即为(dV/dt)/g + θ
	float alloc_diff;         //当前能量分配率 (θ - (dV/dt)/g)
	
	
	float thr_out;    //油门最终输出
	
	float pit_out;    //pit角度最终输出
	
} TECS_Data_structure;

extern TECS_Data_structure tecs_data;

void TECS_Ctrl(float dT_s);

#endif
