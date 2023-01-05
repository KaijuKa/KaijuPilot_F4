#ifndef __ATT_CTRL_H__
#define __ATT_CTRL_H__

#include "stm32f4xx.h"
#include "kaiju_math.h"

extern PID_VAL_structure rol_val_L1;
extern PID_VAL_structure pit_val_L1;
	
extern PID_VAL_structure rol_val_L2;
extern PID_VAL_structure pit_val_L2;

//滚转控制结构体
typedef struct{
	float w;           //期望姿态四元数
	float x;
	float y;
	float z;
	
	float w_err;       //误差四元数
	float x_err;
	float y_err;
	float z_err;
	
	float w_last;      //上次期望姿态四元数
	float x_last;
	float y_last;
	float z_last;
	
	float expect_rol_spd_lpf; //滤波后期望角速度
	float expect_pit_spd_lpf;
	
	float expect_pit_err;     //每次解算误差四元数后的误差角
	float expect_rol_err;
} Rotation_structure;

extern Rotation_structure rotation_data;

void ATT_Ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en);
void Rotation_Ctrl(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void Rotation_Ctrl2(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void ATT_VAL_Init(void);

#endif
