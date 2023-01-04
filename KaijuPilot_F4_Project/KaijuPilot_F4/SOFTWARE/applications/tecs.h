#ifndef __TECS_H__
#define __TECS_H__

#include "stm32f4xx.h"

//tecs���ƽṹ��
typedef struct{
	float air_speed;          //��ǰ���� V
	float altitude;           //��ǰ���θ߶� H
	float track_angle;        //��ǰ������ ��
	float air_spd_diff;       //��ǰ����΢�� dV/dt
	
	float target_track_angle; //Ŀ�꺽����
	float target_air_spd_diff;//Ŀ�����΢��
	
	float target_energy_diff; //Ŀ������΢��
	float target_alloc_diff;  //Ŀ������������
	
	float energy_diff;        //��ǰ����΢�� ((dV/dt)/g + ��)*V Ѳ��V�������� 
							  //���ݴ����к���V ��Ϊ(dV/dt)/g + ��
	float alloc_diff;         //��ǰ���������� (�� - (dV/dt)/g)
	
	
	float thr_out;    //�����������
	
	float pit_out;    //pit�Ƕ��������
	
} TECS_Data_structure;

void TECS_Ctrl(u8 dT_s);

#endif
