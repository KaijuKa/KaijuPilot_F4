#include "route_ctrl.h"
#include "pos_calcu.h"
#include "kaiju_math.h"


/*
  | vel_vec
  |
  |
  |
  ����������������>
      vel_vert_vec
*/


/*******************************************************************************
* �� �� ��         : Route_Ctrl
* ��������		   : ·������
* ��    ��         : ����ms Ŀ�꾭γ��
* ��    ��         : rol�Ƕ�
*******************************************************************************/
float Route_Ctrl(u8 dT_ms, float target_log, float target_lat)
{
	float rtl_vec[2]; //��,γλ��
	float vel_vec[2]; //��,���ٶ�
	float vel_vert_vec[2];//��ֱ���ٶ�����������
	float vel_abs;    //����ֵ�ٶ�
	float dist;       //�������
	float cos_angle, sin_angle;
	float centri_acc;
	s8 rol_direct;        //ȷ�Ϲ�ת����
	static float rol_target;
	
	
	//��ǰλ�õ�Ŀ��λ�õ�����
	rtl_vec[0] = pos_data.log - target_log;
	rtl_vec[1] = pos_data.lat - target_lat;
	
	//��λ����Ϊm
	rtl_vec[0] *= LOG_PARA;
	rtl_vec[1] *= LAT_PARA;
	
	//�ٶ�����
	vel_vec[0] = pos_data.velE;
	vel_vec[1] = pos_data.velN;
	
	//��λ����Ϊm/s
	vel_vec[0] *= 0.01f;
	vel_vec[1] *= 0.01f;
	
	//������ֱ����
	vel_vert_vec[0] = vel_vec[1];
	vel_vert_vec[1] = -vel_vec[0];
	
	//��ά�ٶ� ����
	vel_abs = pos_data.gSpeed;
	
	//��λ����Ϊm/s
	vel_abs *= 0.01f;
	
	//ȷ�Ϲ�ת����
	//Ŀ��ص����˶������Ҳ� ����ת
	if(vel_vert_vec[0] * rtl_vec[0] + vel_vert_vec[1] * rtl_vec[1] >= 0)
	{
		rol_direct = 1;
	}
	//Ŀ��ص����˶�������� ����ת
	else
	{
		rol_direct = -1;
	}
	
	//��Ŀ��ص�ľ������
	dist = my_sqrt(my_pow(rtl_vec[0]) + my_pow(rtl_vec[1]));
	
	//�ٶ���λ�������ļнǵ�cosֵ
	cos_angle = (rtl_vec[0]*vel_vec[0] + rtl_vec[1]*vel_vec[1]) * safe_div(1, (vel_abs * dist), 0);
		
	//�нǵ�sinֵ
	sin_angle = my_sqrt(1 - my_pow(cos_angle));
	
	//��ת���������ļ��ٶ�
	centri_acc = 2 * vel_abs * vel_abs * sin_angle * safe_div(1, dist, 0);
	
	//������rol�Ƕ� �˲�
	rol_target += 0.5f * (rol_target - rol_direct*fast_atan2(centri_acc, 9.81f)*57.30f);
	
	//40������
	rol_target = LIMIT(rol_target, -40, 40);
	
	return rol_target;
}
