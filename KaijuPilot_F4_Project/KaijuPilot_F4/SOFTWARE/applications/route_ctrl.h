#ifndef __ROUTE_CTRL_H__
#define __ROUTE_CTRL_H__

#include "stm32f4xx.h"

#define LOG_PARA 10000.0f
#define LAT_PARA 11000.0f

float Route_Ctrl(u8 dT_ms, float target_log, float target_lat);

#endif
