#ifndef __ARG_MANAGE_H__
#define __ARG_MANAGE_H__

#include "io.h"

typedef struct{
	float pit_offset; //imu pit rolˮƽУ׼ƫ��
	float rol_offset;
	
	u8 ch1_direct;    //ͨ��124 �������
	u8 ch2_direct;
	u8 ch4_direct;
	
	s16 ch1_offset;   //ͨ��1234�����ƫ��
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	u8 ratio_rol;     //pit rol�ж� 50Ϊ1
	u8 ratio_pit; 
	
	u8 ctrl_mode;     //�̶���ģʽ 0������ 1��ͳ����
	
	u8 rol_angle_max; //����ģʽ�� �����б�Ƕ� ��λ��
	u8 pit_angle_max;
	
					  //���ֶ�ģʽ�� �����ת�ٶ� ��λ��/s
	u16 rol_angular_spd_max;
	u16 pit_angular_spd_max;
	
	
} arg_st;

typedef union{
	arg_st arg;
	u8 raw_data[40];
} arg_union;


extern arg_st flight_arg;
extern arg_union storaged_arg;

void arg_load(void);
void arg_store(void);
void argSet_byteGet(u8 data);
#endif
