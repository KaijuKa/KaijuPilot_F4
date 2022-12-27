#ifndef __ATT_CTRL_H__
#define __ATT_CTRL_H__

#include "io.h"
#include "kaiju_math.h"

extern PID_VAL_structure rol_val_L1;
extern PID_VAL_structure pit_val_L1;
	
extern PID_VAL_structure rol_val_L2;
extern PID_VAL_structure pit_val_L2;

void att_ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en);
void att_val_init(void);

#endif
