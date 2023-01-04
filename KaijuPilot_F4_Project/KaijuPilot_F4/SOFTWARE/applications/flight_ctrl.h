#ifndef __FLIGHT_CTRL_H__
#define __FLIGHT_CTRL_H__

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

//飞行数据结构体
typedef struct{
	u8 flight_mode;
	u8 flight_stat;
	
	s16 pwm_out[4];
} FLIGHT_Data_structure;

enum
{
	Manual,    //手动模式
	Stabilize, //自稳模式
	Sport,     //运动模式
	RTL,       //返航模式
	MODE_NUM
};

extern FLIGHT_Data_structure fl_data;

extern QueueHandle_t fl_data_queue;

void Flight_Data_Share(void);
void Flight_Ctrl_Task(u8 dT_ms);
void Manual_Task(void);
void Stabilize_Task(u8 dT_ms);
void Sport_Task(u8 dT_ms);
#endif
