#ifndef __POS_CALCU_H__
#define __POS_CALCU_H__

#include "stm32f4xx.h"

typedef struct{
	float baro_height;
	float baro_height_err;
	float fusion_height;
} POS_Structure;

extern POS_Structure pos_data;

void POS_Init(void);
void RAW_Height_Calibration(void);
void POS_Update(u8 dT_ms);
void RAW_Height_Update(u8 dT_ms);
void Height_Fusion(u8 dT_ms);

#endif
