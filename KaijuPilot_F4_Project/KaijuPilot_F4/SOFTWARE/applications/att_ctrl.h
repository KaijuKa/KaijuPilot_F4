#ifndef __ATT_CTRL_H__
#define __ATT_CTRL_H__

#include "stm32f4xx.h"
#include "kaiju_math.h"

extern PID_VAL_structure rol_val_L1;
extern PID_VAL_structure pit_val_L1;
	
extern PID_VAL_structure rol_val_L2;
extern PID_VAL_structure pit_val_L2;

void ATT_Ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en);
void Rotation_Ctrl(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void Rotation_ctrl2(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void ATT_VAL_Init(void);

#endif
