#ifndef __FLIGHT_CTRL_H__
#define __FLIGHT_CTRL_H__

#include "stm32f4xx.h"

typedef struct{
	u8 flight_mode;
	u8 flight_stat;
} FLIGHT_Data_structure;

enum
{
	Manual,
	Stabilize,
	Sport,
	RTL,
	MODE_NUM
};

extern FLIGHT_Data_structure fl_data;
void Flight_Ctrl_Task(u8 dT_ms);
void Manual_Task(void);
void Stabilize_Task(u8 dT_ms);
void Sport_Task(u8 dT_ms);
#endif
