#ifndef __FLIGHT_CTRL_H__
#define __FLIGHT_CTRL_H__

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"

//�������ݽṹ��
typedef struct{
	u8 flight_mode; //����ģʽ
	u8 flight_stat; //����״̬ 0δ���� 1����
	
	u8 pos_stat;    //��λ״̬ 0δ��λ 1��λ
	
	float pos_log;  //��λ֮��ļ�λ�� ����
	float pos_lat;  //��λ֮��ļ�λ�� ��γ
	
	float target_rol;//Ŀ��rol
	float target_pit;//Ŀ��pit
	
	s16 pwm_out[4];
} FLIGHT_Data_structure;

enum
{
	Manual,    //�ֶ�ģʽ
	Stabilize, //����ģʽ
	Sport,     //�˶�ģʽ
	RTL,       //����ģʽ
	MODE_NUM
};

extern FLIGHT_Data_structure fl_data;

extern QueueHandle_t fl_data_queue;

void Flight_Data_Share(void);
void Flight_Ctrl_Task(u8 dT_ms);
void Manual_Task(void);
void Stabilize_Task(u8 dT_ms);
void Sport_Task(u8 dT_ms);
void RTL_Task(u8 dT_ms);
#endif
