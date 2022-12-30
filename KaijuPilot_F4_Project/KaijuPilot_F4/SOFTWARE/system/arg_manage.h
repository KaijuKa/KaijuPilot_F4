#ifndef __ARG_MANAGE_H__
#define __ARG_MANAGE_H__

//#include "io.h"
#include "stm32f4xx.h"

typedef struct{
	float pit_offset; //imu pit rolˮƽУ׼ƫ�� ��λ��
	float rol_offset;
	
	u8 ch1_direct;    //ͨ��124 �������
	u8 ch2_direct;
	u8 ch4_direct;
	
	s16 ch1_offset;   //ͨ��1234�����ƫ��
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	u8 ratio_att_rol;     //pit rol ���ȵ�ģʽ�ж� 50Ϊ1
	u8 ratio_att_pit; 
	
	u8 ratio_rotation_rol; //pit rol �˶�ģʽ�ж� 50Ϊ1
	u8 ratio_rotation_pit;
	
	u8 ctrl_mode;     //�̶���ģʽ 0������ 1��ͳ����
	
	u8 rol_angle_max; //����ģʽ�� �����б�Ƕ� ��λ��
	u8 pit_angle_max;
	
					  //���ֶ�ģʽ�� �����ת�ٶ� ��λ��/s
	u16 rol_angular_spd_max;
	u16 pit_angular_spd_max;
					  //�Զ�Ѳ��ģʽ�� �����趨ֵ �� ���θ߶��趨ֵ
	u8 target_air_speed;      //��λ m/s
	u8 target_autoft_altitude;//��λ m
	
	
} ARG_structure;

extern ARG_structure flight_arg;

#endif
