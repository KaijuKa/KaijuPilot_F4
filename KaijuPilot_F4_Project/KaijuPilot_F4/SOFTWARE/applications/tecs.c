#include "tecs.h"
#include "kaiju_math.h"
#include "arg_manage.h"

#define GRAV_GCC 9.8f //�������ٶ�

TECS_Data_structure tecs_data;

// Ŀ��Ѳ���߶� ת��Ϊ Ŀ�꺽����
PID_ARG_structure alt_2_tt_angle_arg;
PID_VAL_structure alt_2_tt_angle_val;

// Ŀ��Ѳ���ٶ� ת��Ϊ Ŀ��Ѳ���ٶ�΢��
PID_ARG_structure spd_2_t_spd_diff_arg;
PID_VAL_structure spd_2_t_spd_diff_val;

//���ű����������
PID_ARG_structure thr_p_arg;
PID_VAL_structure thr_p_val;

//���Ż���΢�ֿ������
PID_ARG_structure thr_id_arg;
PID_VAL_structure thr_id_val;

//����������������
PID_ARG_structure pit_p_arg;
PID_VAL_structure pit_p_val;

//���������΢�ֿ������
PID_ARG_structure pit_id_arg;
PID_VAL_structure pit_id_val;

/*******************************************************************************
* �� �� ��         : TECS_Ctrl
* ��������		     : TECS����
* ��    ��         : ����dT_s
* ��    ��         : ��
*******************************************************************************/
void TECS_Ctrl(u8 dT_s)
{
	//����Ŀ�꺽����
	pid_calcu(dT_s, flight_arg.target_autoft_altitude, tecs_data.altitude, 
	&alt_2_tt_angle_arg, &alt_2_tt_angle_val, 0, 0);
	tecs_data.target_track_angle = alt_2_tt_angle_val.out / tecs_data.air_speed;
	
	//����Ŀ�����΢��
	pid_calcu(dT_s, flight_arg.target_air_speed, tecs_data.air_speed, 
	&spd_2_t_spd_diff_arg, &spd_2_t_spd_diff_val, 0, 0);
	tecs_data.target_air_spd_diff = alt_2_tt_angle_val.out;
	
	//����Ŀ������΢��
	tecs_data.target_energy_diff = tecs_data.target_air_spd_diff/GRAV_GCC + tecs_data.target_track_angle;
	
	//����Ŀ������������
	tecs_data.target_alloc_diff = tecs_data.target_track_angle - tecs_data.target_air_spd_diff/GRAV_GCC;
	
	//���㵱ǰ����΢��
	tecs_data.energy_diff = tecs_data.air_spd_diff/GRAV_GCC + tecs_data.track_angle;
	
	//���㵱ǰ����������
	tecs_data.alloc_diff = tecs_data.track_angle - tecs_data.air_spd_diff/GRAV_GCC;
	
	//�������ű���������
	pid_calcu(dT_s, tecs_data.energy_diff, 0, &thr_p_arg, &thr_p_val, 0, 0);
	
	//�������ű���������
	pid_calcu(dT_s, tecs_data.alloc_diff, 0, &pit_p_arg, &pit_p_val, 0, 0);
	
	//�������Ż���΢�ֿ�����
	pid_calcu(dT_s, tecs_data.target_energy_diff, tecs_data.energy_diff, &thr_id_arg, &thr_id_val, 200, 1);
	
	//�������ű���������
	pid_calcu(dT_s, tecs_data.target_alloc_diff, tecs_data.alloc_diff, &pit_id_arg, &pit_id_val, 200, 1);
	
	//�����������
	tecs_data.thr_out = thr_p_val.out + thr_id_val.out;
	tecs_data.pit_out = pit_p_val.out + pit_id_val.out;
}
