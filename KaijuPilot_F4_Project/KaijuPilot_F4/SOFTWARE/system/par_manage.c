#include "par_manage.h"
#include "at24c02.h"

PAR_Store_union storaged_par;

PAR_INFO_structure par_info_array[PAR_NUM] = {
	{.par_max = 9000, .par_min = -9000, .name = "pit_offset         ", .info = "设置俯仰偏置 度*100"},
	{.par_max = 9000, .par_min = -9000, .name = "rol_offset         ", .info = "设置横滚偏置 度*100"},
	{.par_max = 1, .par_min = 0, .name = "ch1_direct         ", .info = "设置PWM1方向       "},
	{.par_max = 1, .par_min = 0, .name = "ch2_direct         ", .info = "设置PWM2方向       "},
	{.par_max = 1, .par_min = 0, .name = "ch4_direct         ", .info = "设置PWM4方向       "},
	{.par_max = 100, .par_min = -100, .name = "ch1_offset         ", .info = "设置PWM1偏置       "},
	{.par_max = 100, .par_min = -100, .name = "ch2_offset         ", .info = "设置PWM2偏置       "},
	{.par_max = 100, .par_min = -100, .name = "ch3_offset         ", .info = "设置PWM3偏置       "},
	{.par_max = 100, .par_min = -100, .name = "ch4_offset         ", .info = "设置PWM4偏置       "},
	{.par_max = 50, .par_min = 1, .name = "ratio_att_rol      ", .info = "设置横滚感度 50为1 "},
	{.par_max = 50, .par_min = 1, .name = "ratio_att_pit      ", .info = "设置俯仰感度 50为1 "},
	{.par_max = 50, .par_min = 1, .name = "ratio_rotation_rol ", .info = "设置sp横滚感度     "},
	{.par_max = 50, .par_min = 1, .name = "ratio_rotation_rol ", .info = "设置sp俯仰感度     "},
	{.par_max = 1, .par_min = 0, .name = "ctrl_mode          ", .info = "1传统布局 0三角    "},
	{.par_max = 90, .par_min = 0, .name = "rol_angle_max      ", .info = "最大横滚角 度      "},
	{.par_max = 90, .par_min = 0, .name = "pit_angle_max      ", .info = "最大俯仰角 度      "},
	{.par_max = 300, .par_min = 0, .name = "rol_angular_spd_max", .info = "最大横滚角速度 度/s"},
	{.par_max = 300, .par_min = 0, .name = "pit_angular_spd_max", .info = "最大俯仰角速度 度/s"},
	{.par_max = 50, .par_min = 10, .name = "target_air_speed   ", .info = "目标空速m/s        "},
	{.par_max = 100, .par_min = 10, .name = "target_autoft_alt  ", .info = "目标高度m          "},
	{.par_max = 100, .par_min = 0, .name = "auto_thr_max       ", .info = "最大自动油门%      "},
	{.par_max = 100, .par_min = 0, .name = "auto_thr_min       ", .info = "最小自动油门%      "},
	{.par_max = 90, .par_min = 0, .name = "auto_pit_max       ", .info = "最大俯仰角 度      "}
};

PAR_MSG_union fl_par = { 
	.par = {
		.pit_offset = 0,
		.rol_offset = 0,
	
		.ch1_direct = 0,
		.ch2_direct = 1,
		.ch4_direct = 0,
	
		.ch1_offset = 0,
		.ch2_offset = 0,
		.ch3_offset = 0,
		.ch4_offset = 0,
	
		.ratio_att_rol = 21,
		.ratio_att_pit = 50,
		
		.ratio_rotation_rol = 21,
		.ratio_rotation_pit = 50,
	
		.ctrl_mode = 1,
	
		.rol_angle_max = 65,
		.pit_angle_max = 50,
	
		.rol_angular_spd_max = 100,
		.pit_angular_spd_max = 80,
		
		.target_air_speed = 15,
		.target_autoft_altitude = 20,
		.auto_thr_max = 60,
		.auto_thr_min = 40,
		.auto_pit_max = 40
	}
};

u8 is_par_dirty;
u8 is_start_store;

/*******************************************************************************
* 函 数 名         : PAR_Load
* 函数功能		     : 从AT24中读入参数列表
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void PAR_Load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//读入参数
	DRV_AT24_Read_Str(storaged_par.raw_data, LOAD_MAX, 0);
	
	//校验
	for(i = 0; i < LOAD_MAX-1; i++)
	{
		tmp += storaged_par.raw_data[i];
	}
	
	if(tmp == storaged_par.raw_data[LOAD_MAX-1])
	{
		//赋值
		fl_par.par = storaged_par.par;
	}
}

/*******************************************************************************
* 函 数 名         : PAR_Store
* 函数功能		     : 加载所有参数到AT24
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void PAR_Store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_par.par = fl_par.par;
	
	//计算校验
	for(i = 0; i < LOAD_MAX-1; i++)
	{
		tmp += storaged_par.raw_data[i];
	}
	storaged_par.raw_data[LOAD_MAX-1] = tmp;
	
	//写入AT24
	DRV_AT24_Write_Str(storaged_par.raw_data, LOAD_MAX, 0);
}

/*******************************************************************************
* 函 数 名         : PAR_Change
* 函数功能		     : 修改参数
* 输    入         : 参数id, 参数值
* 输    出         : 无
*******************************************************************************/
void PAR_Change(u8 par_id, float par_val)
{
	static u8 cnt = 0;
	
	//接收完毕所有参数进行保存
	if(cnt < PAR_NUM)
	{
		cnt ++;
	}
	else
	{
		cnt = 0;
		is_start_store = 1;
	}
	
	//参数没有变化 退出
	if(fl_par.s16_unit_array[par_id] == par_val)
	{
		return;
	}
	else 
	{
		//参数不符合规定 退出
		if(par_val < par_info_array[par_id].par_min || par_val > par_info_array[par_id].par_max)
		{
			return;
		}
		else
		{
			//修改参数并标记已修改
			fl_par.s16_unit_array[par_id] = (s16)par_val;
			is_par_dirty = 1;
		}
	}
}

/*******************************************************************************
* 函 数 名         : PAR_Store_Task
* 函数功能		     : 参数保存任务
* 输    入         : dT_ms
* 输    出         : 无
*******************************************************************************/
void PAR_Store_Task(u8 dT_ms)
{
	static u16 time_cnt;
	//可以开始保存
	if(is_start_store == 1)
	{
		//没有参数修改 复位标志
		if(is_par_dirty == 0)
		{
			is_start_store = 0;
		}
		//有参数修改 保存并复位标志
		else
		{
			//等待一段时间
			if(time_cnt < STORE_DELAY)
			{
				time_cnt += dT_ms;
			}
			else
			{
				time_cnt = 0;
				PAR_Store();
				is_start_store = 0;
				is_par_dirty = 0;
			}
		}
	}
}


