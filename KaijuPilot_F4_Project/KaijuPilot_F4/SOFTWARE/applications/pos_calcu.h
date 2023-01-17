#ifndef __POS_CALCU_H__
#define __POS_CALCU_H__

#include "stm32f4xx.h"

typedef struct{
	float baro_height;        //cm
	float baro_height_offset;
	float fusion_height;
	float baro_velD;       //��ѹ�Ƶõ����ٶ�cm/s
	
	u8 fix_sta;            //��λ״̬
	u8 star_num;           //������
	float log;             //deg  ����
	float lat;             //deg  ά��
	float hMSL;            //cm   ����ƽ��ˮƽ��߶�
	float hAcc;            //cm   ˮƽ���ȹ���
	float cAcc;            //cm   ��ֱ���ȹ���
	float velN;            //cm/s ���ٶ�
	float velE;            //cm/s ���ٶ�
	float gSpeed;          //cm/s ����
	float spdAcc;          //cm/s �ٶȾ��ȹ���
	float headMot;         //deg  2D�˶�����
	float headAcc;         //deg  �˶����򾫶ȹ���
	float pDop;            //λ�þ������� �ڷɿ��ϸ�ֵС��4��ʾ����
} POS_Structure;

extern POS_Structure pos_data;

void POS_Init(void);
void RAW_Height_Calibration(void);
void POS_Update(u8 dT_ms);
void RAW_Height_Update(u8 dT_ms);
void Height_Fusion(u8 dT_ms);
#endif
