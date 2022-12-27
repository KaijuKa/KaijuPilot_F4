#ifndef __FLIGHT_CTRL_H__
#define __FLIGHT_CTRL_H__

#include "io.h"

typedef struct{
	u8 flight_mode;
	u8 flight_stat;
} FLIGHT_CTRL_ST;

enum
{
	Manual,
	Stabilize,
	Sport,
	RTL,
	MODE_NUM
};

extern FLIGHT_CTRL_ST fl_st;
void flight_ctrl_task(void);
void Manual_task(void);
void Stabilize_task(void);
#endif
