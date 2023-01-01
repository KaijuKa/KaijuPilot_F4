#ifndef __ARG_MANAGE_H__
#define __ARG_MANAGE_H__

//#include "io.h"
#include "stm32f4xx.h"

#define LOAD_MAX 64 //参数列表在eeprom里的大小

typedef struct{
	float pit_offset; //imu pit rol水平校准偏置 单位度 0-90
	float rol_offset;
	
	u8 ch1_direct;    //通道124 输出正反 1/0
	u8 ch2_direct;
	u8 ch4_direct;
	
	s16 ch1_offset;   //通道1234输出的偏置 直接参与pwm输出 0-100
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	u8 ratio_att_rol;     //pit rol 自稳等模式感度 50为1
	u8 ratio_att_pit; 
	
	u8 ratio_rotation_rol; //pit rol 运动模式感度 50为1
	u8 ratio_rotation_pit;
	
	u8 ctrl_mode;     //固定翼模式 0三角翼 1传统布局
	
	u8 rol_angle_max; //自稳模式下 最大倾斜角度 单位度 0-90
	u8 pit_angle_max;                                  
	
					  //除手动模式下 最大旋转速度 单位度/s
	u16 rol_angular_spd_max;
	u16 pit_angular_spd_max;
	
					  //自动巡航模式下 空速设定值 和 海拔高度设定值
	u8 target_air_speed;      //单位 m/s  0-127
	u8 target_autoft_altitude;//单位 m    0-127
	u8 auto_thr_max;          //最大油门  0-100
	u8 auto_thr_min;          //最小油门  0-100
	u8 auto_pit_max;          //pit限制值 0-90
	
} ARG_structure;

typedef union{
	ARG_structure arg;
	u8 raw_data[LOAD_MAX];
} ARG_union;

extern ARG_structure flight_arg;
extern ARG_union storaged_arg;

void ARG_Load(void);
void ARG_Store(void);

#endif
