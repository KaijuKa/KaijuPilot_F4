#include "tecs.h"
#include "kaiju_math.h"
#include "arg_manage.h"

#define GRAV_GCC 9.8f //重力加速度

TECS_Data_structure tecs_data;

// 目标巡航高度 转换为 目标航迹角
PID_ARG_structure alt_2_tt_angle_arg;
PID_VAL_structure alt_2_tt_angle_val;

// 目标巡航速度 转换为 目标巡航速度微分
PID_ARG_structure spd_2_t_spd_diff_arg;
PID_VAL_structure spd_2_t_spd_diff_val;

//油门比例控制相关
PID_ARG_structure thr_p_arg;
PID_VAL_structure thr_p_val;

//油门积分微分控制相关
PID_ARG_structure thr_id_arg;
PID_VAL_structure thr_id_val;

//升降舵比例控制相关
PID_ARG_structure pit_p_arg;
PID_VAL_structure pit_p_val;

//升降舵积分微分控制相关
PID_ARG_structure pit_id_arg;
PID_VAL_structure pit_id_val;

/*******************************************************************************
* 函 数 名         : TECS_Ctrl
* 函数功能		     : TECS控制
* 输    入         : 周期dT_s
* 输    出         : 无
*******************************************************************************/
void TECS_Ctrl(u8 dT_s)
{
	//计算目标航迹角
	pid_calcu(dT_s, flight_arg.target_autoft_altitude, tecs_data.altitude, 
	&alt_2_tt_angle_arg, &alt_2_tt_angle_val, 0, 0);
	tecs_data.target_track_angle = alt_2_tt_angle_val.out / tecs_data.air_speed;
	
	//计算目标空速微分
	pid_calcu(dT_s, flight_arg.target_air_speed, tecs_data.air_speed, 
	&spd_2_t_spd_diff_arg, &spd_2_t_spd_diff_val, 0, 0);
	tecs_data.target_air_spd_diff = alt_2_tt_angle_val.out;
	
	//计算目标能量微分
	tecs_data.target_energy_diff = tecs_data.target_air_spd_diff/GRAV_GCC + tecs_data.target_track_angle;
	
	//计算目标能量分配率
	tecs_data.target_alloc_diff = tecs_data.target_track_angle - tecs_data.target_air_spd_diff/GRAV_GCC;
	
	//计算当前能量微分
	tecs_data.energy_diff = tecs_data.air_spd_diff/GRAV_GCC + tecs_data.track_angle;
	
	//计算当前能量分配率
	tecs_data.alloc_diff = tecs_data.track_angle - tecs_data.air_spd_diff/GRAV_GCC;
	
	//计算油门比例控制项
	pid_calcu(dT_s, tecs_data.energy_diff, 0, &thr_p_arg, &thr_p_val, 0, 0);
	
	//计算油门比例控制项
	pid_calcu(dT_s, tecs_data.alloc_diff, 0, &pit_p_arg, &pit_p_val, 0, 0);
	
	//计算油门积分微分控制项
	pid_calcu(dT_s, tecs_data.target_energy_diff, tecs_data.energy_diff, &thr_id_arg, &thr_id_val, 200, 1);
	
	//计算油门比例控制项
	pid_calcu(dT_s, tecs_data.target_alloc_diff, tecs_data.alloc_diff, &pit_id_arg, &pit_id_val, 200, 1);
	
	//计算最终输出
	tecs_data.thr_out = thr_p_val.out + thr_id_val.out;
	tecs_data.pit_out = pit_p_val.out + pit_id_val.out;
}
