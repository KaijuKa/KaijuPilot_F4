#ifndef __MY_LINETRACKING_H__
#define __MY_LINETRACKING_H__


#include "io.h"
#include "delay.h"

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;
	
	u8 point_type;
	s16 point_pos_x;
	s16 point_pos_y;
	
	u8 offline;

}_openmv_line_tracking_st;

typedef struct{
	u8 target_loss_uflag;        //Ŀ�궪ʧ���
	s16 post_xy[2];              //opmv�����λ����Ϣ
	float post_final[2][2];      //�����˲��͵���������λ����Ϣ
	
	float post_reality_cm[2][2]; //ʵ���е�λ����Ϣ
	float post_err_diff[2];      //λ����Ϣ΢��
	float post_err_i;            //������
	float output_final_PDF[2];   //����PDF����Ľ��
} opmv_lt2;

extern float angle;
extern opmv_lt2 my_opmv_lt2;
extern s16 pos_x_err_ct;              //����ʱx����ƫ����
extern s16 pos_y_err_ct;              //����ʱy����ƫ����
extern s16 track_vel;
extern _openmv_line_tracking_st my_opmv;

void opmv_ct_Decoupling(void);
void opmv_ct_Calculating(u8 dT_ms);
void opmv_lt2_Ctrl(u8 uflag);
void opmv_lt2_ctrl_task(u8 dT_ms);
#endif
