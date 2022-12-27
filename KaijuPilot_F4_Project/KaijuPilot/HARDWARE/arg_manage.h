#ifndef __ARG_MANAGE_H__
#define __ARG_MANAGE_H__

#include "io.h"

typedef struct{
	float pit_offset; //imu pit rol水平校准偏置
	float rol_offset;
	
	u8 ch1_direct;    //通道124 输出正反
	u8 ch2_direct;
	u8 ch4_direct;
	
	s16 ch1_offset;   //通道1234输出的偏置
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	u8 ratio_rol;     //pit rol感度 50为1
	u8 ratio_pit; 
	
	u8 ctrl_mode;     //固定翼模式 0三角翼 1传统布局
	
	u8 rol_angle_max; //自稳模式下 最大倾斜角度 单位度
	u8 pit_angle_max;
	
					  //除手动模式下 最大旋转速度 单位度/s
	u16 rol_angular_spd_max;
	u16 pit_angular_spd_max;
	
	
} arg_st;

typedef union{
	arg_st arg;
	u8 raw_data[40];
} arg_union;


extern arg_st flight_arg;
extern arg_union storaged_arg;

void arg_load(void);
void arg_store(void);
void argSet_byteGet(u8 data);
#endif
