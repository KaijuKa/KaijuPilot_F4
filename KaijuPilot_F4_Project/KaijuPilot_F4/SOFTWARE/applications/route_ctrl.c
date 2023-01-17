#include "route_ctrl.h"
#include "pos_calcu.h"
#include "kaiju_math.h"


/*
  | vel_vec
  |
  |
  |
  ————————>
      vel_vert_vec
*/


/*******************************************************************************
* 函 数 名         : Route_Ctrl
* 函数功能		   : 路径控制
* 输    入         : 周期ms 目标经纬度
* 输    出         : rol角度
*******************************************************************************/
float Route_Ctrl(u8 dT_ms, float target_log, float target_lat)
{
	float rtl_vec[2]; //经,纬位置
	float vel_vec[2]; //东,北速度
	float vel_vert_vec[2];//垂直于速度向量的向量
	float vel_abs;    //绝对值速度
	float dist;       //两点距离
	float cos_angle, sin_angle;
	float centri_acc;
	s8 rol_direct;        //确认滚转方向
	static float rol_target;
	
	
	//当前位置到目标位置的向量
	rtl_vec[0] = pos_data.log - target_log;
	rtl_vec[1] = pos_data.lat - target_lat;
	
	//单位换算为m
	rtl_vec[0] *= LOG_PARA;
	rtl_vec[1] *= LAT_PARA;
	
	//速度向量
	vel_vec[0] = pos_data.velE;
	vel_vec[1] = pos_data.velN;
	
	//单位换算为m/s
	vel_vec[0] *= 0.01f;
	vel_vec[1] *= 0.01f;
	
	//构建垂直向量
	vel_vert_vec[0] = vel_vec[1];
	vel_vert_vec[1] = -vel_vec[0];
	
	//二维速度 地速
	vel_abs = pos_data.gSpeed;
	
	//单位换算为m/s
	vel_abs *= 0.01f;
	
	//确认滚转方向
	//目标地点在运动方向右侧 正滚转
	if(vel_vert_vec[0] * rtl_vec[0] + vel_vert_vec[1] * rtl_vec[1] >= 0)
	{
		rol_direct = 1;
	}
	//目标地点在运动方向左侧 负滚转
	else
	{
		rol_direct = -1;
	}
	
	//到目标地点的距离计算
	dist = my_sqrt(my_pow(rtl_vec[0]) + my_pow(rtl_vec[1]));
	
	//速度与位置向量的夹角的cos值
	cos_angle = (rtl_vec[0]*vel_vec[0] + rtl_vec[1]*vel_vec[1]) * safe_div(1, (vel_abs * dist), 0);
		
	//夹角的sin值
	sin_angle = my_sqrt(1 - my_pow(cos_angle));
	
	//滚转期望的向心加速度
	centri_acc = 2 * vel_abs * vel_abs * sin_angle * safe_div(1, dist, 0);
	
	//期望的rol角度 滤波
	rol_target += 0.5f * (rol_target - rol_direct*fast_atan2(centri_acc, 9.81f)*57.30f);
	
	//40度限制
	rol_target = LIMIT(rol_target, -40, 40);
	
	return rol_target;
}
