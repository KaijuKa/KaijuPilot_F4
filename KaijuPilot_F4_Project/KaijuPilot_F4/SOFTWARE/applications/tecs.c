#include "tecs.h"
#include "kaiju_math.h"
#include "par_manage.h"
#include "pos_calcu.h"
#include "imu.h"

#define GRAV_GCC 9.8f //�������ٶ�

TECS_Data_structure tecs_data;

// Ŀ��Ѳ���߶� ת��Ϊ Ŀ�꺽����
PID_ARG_structure alt_2_tt_angle_arg = {
	.kp = 1.5f,
	.kd = 0,
	.ki = 0,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure alt_2_tt_angle_val;

// Ŀ��Ѳ���ٶ� ת��Ϊ Ŀ��Ѳ���ٶ�΢��
PID_ARG_structure spd_2_t_spd_diff_arg = {
	.kp = 50.0f,
	.kd = 0,
	.ki = 0,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure spd_2_t_spd_diff_val;

//���ű����������
PID_ARG_structure thr_p_arg = {
	.kp = 1.2f,
	.kd = 0,
	.ki = 0,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure thr_p_val;

//���Ż���΢�ֿ������
PID_ARG_structure thr_id_arg = {
	.kp = 0,
	.kd = 0,
	.ki = 0.06f,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure thr_id_val;

//����������������
PID_ARG_structure pit_p_arg = {
	.kp = 1.2f,
	.kd = 0,
	.ki = 0,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure pit_p_val;

//���������΢�ֿ������
PID_ARG_structure pit_id_arg = {
	.kp = 0,
	.kd = 0,
	.ki = 0.06f,
	.expect_kd = 0,
	.fb_kd = 0
};
PID_VAL_structure pit_id_val;

/*******************************************************************************
* �� �� ��         : TECS_Ctrl
* ��������		     : TECS����
* ��    ��         : ����dT_s
* ��    ��         : ��
*******************************************************************************/
void TECS_Ctrl(float dT_s)
{
	static float last_air_speed;
	
	IMU_Data_structure imu_data;
	//����Ϣ������ȡ��imu����
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);
	
	tecs_data.air_speed = pos_data.gSpeed;
	tecs_data.altitude = pos_data.fusion_height;
	tecs_data.track_angle = imu_data.pit;
	tecs_data.air_spd_diff = (tecs_data.air_speed - last_air_speed)/dT_s;
	last_air_speed = tecs_data.air_speed;
	
	
	//����Ŀ�꺽����
	pid_calcu(dT_s, fl_par.par.target_autoft_altitude, tecs_data.altitude, 
	&alt_2_tt_angle_arg, &alt_2_tt_angle_val, 0, 0);
	tecs_data.target_track_angle = alt_2_tt_angle_val.out / tecs_data.air_speed;
	
	//����Ŀ�����΢��
	pid_calcu(dT_s, fl_par.par.target_air_speed, tecs_data.air_speed, 
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
	
	//���Ʒ���
	//thr �Ŵ�pwmֵ
	tecs_data.thr_out = LIMIT(tecs_data.thr_out, fl_par.par.auto_thr_min, fl_par.par.auto_thr_max);
	tecs_data.thr_out *= 4.0f;
	tecs_data.pit_out = LIMIT(tecs_data.pit_out, -fl_par.par.auto_pit_max, fl_par.par.auto_pit_max);
	
}
