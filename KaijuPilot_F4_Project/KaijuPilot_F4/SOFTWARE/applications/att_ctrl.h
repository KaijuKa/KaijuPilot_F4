#ifndef __ATT_CTRL_H__
#define __ATT_CTRL_H__

#include "stm32f4xx.h"
#include "kaiju_math.h"

extern PID_VAL_structure rol_val_L1;
extern PID_VAL_structure pit_val_L1;
	
extern PID_VAL_structure rol_val_L2;
extern PID_VAL_structure pit_val_L2;

//��ת���ƽṹ��
typedef struct{
	float w;           //������̬��Ԫ��
	float x;
	float y;
	float z;
	
	float w_err;       //�����Ԫ��
	float x_err;
	float y_err;
	float z_err;
	
	float w_last;      //�ϴ�������̬��Ԫ��
	float x_last;
	float y_last;
	float z_last;
	
	float expect_rol_spd_lpf; //�˲����������ٶ�
	float expect_pit_spd_lpf;
	
	float expect_pit_err;     //ÿ�ν��������Ԫ���������
	float expect_rol_err;
} Rotation_structure;

extern Rotation_structure rotation_data;

void ATT_Ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en);
void Rotation_Ctrl(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void Rotation_Ctrl2(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en);
void ATT_VAL_Init(void);

#endif
