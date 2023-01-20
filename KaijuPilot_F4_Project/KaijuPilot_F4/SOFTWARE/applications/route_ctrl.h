#ifndef __ROUTE_CTRL_H__
#define __ROUTE_CTRL_H__

#include "stm32f4xx.h"

typedef struct{
	float dist;
	float vel_abs;
	float angle;
	float centri_acc;
	u8 direct;
}	ROUTE_Data_Structure;

#define LOG_PARA 111194.92f
#define LAT_PARA 100804.60f

extern ROUTE_Data_Structure route_data;

float Route_Ctrl(u8 dT_ms, float target_log, float target_lat);

#endif
