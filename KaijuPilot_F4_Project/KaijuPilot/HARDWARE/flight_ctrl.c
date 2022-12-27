#include "flight_ctrl.h"
#include "remote_signal.h"
#include "pwm.h"
#include "arg_manage.h"
#include "imu.h"
#include "att_ctrl.h"

FLIGHT_CTRL_ST fl_st;

/*******************************************************************************
* 函 数 名         : flight_ctrl_task
* 函数功能		     : 飞行控制总管任务
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void flight_ctrl_task(void)
{
	//根据通道5得到当前模式
	if(rc_in.ch_processed[AUX1] > 300)
	{
		fl_st.flight_mode = Manual;
	}
	else
	{
		fl_st.flight_mode = Stabilize;
	}
	
//	if(rc_in.ch_processed[AUX1] > 300)
//	{
//		fl_st.flight_mode = Manual;
//	}
//	else if(rc_in.ch_processed[AUX1] > -300)
//	{
//		fl_st.flight_mode = Stabilize;
//	}
//	else
//	{
//		fl_st.flight_mode = RTL;
//	}
	
	//无ppm信号默认返航
	if(rc_in.ppm_offline > 1)
	{
		fl_st.flight_mode = Manual;
	}
	
	//有油门 表示起飞 开始积分
	if(rc_in.ch_processed[CH_THR] > -400)
	{
		fl_st.flight_stat = 1;
	}
	else
	{
		fl_st.flight_stat = 0;
	}
	
	//根据飞行模式执行相应操作
	switch(fl_st.flight_mode)
	{
		case Manual:
		{
			att_val_init();
			Manual_task();
			break;
		}
		case Stabilize:
		{
			Stabilize_task();
			break;
		}
		case Sport:
		{
			break;
		}
		case RTL:
		{
			break;
		}
	}
}

/*******************************************************************************
* 函 数 名         : Manual_task
* 函数功能		     : 手动模式下的外设控制
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void Manual_task(void)
{
	pwm_output(rc_in.ch_processed[CH_ROL], rc_in.ch_processed[CH_PIT],\
	rc_in.ch_processed[CH_THR], rc_in.ch_processed[CH_YAW]);
}

/*******************************************************************************
* 函 数 名         : Stabilize_task
* 函数功能		     : 自稳模式下的外设控制
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void Stabilize_task(void)
{
	float expect_rol = (rc_in.ch_processed[CH_ROL]/500.0f)*flight_arg.rol_angle_max;
	float expect_pit = (rc_in.ch_processed[CH_PIT]/500.0f)*flight_arg.pit_angle_max;
	
	att_ctrl(0.02f, expect_rol, expect_pit, fl_st.flight_stat);
	
	pwm_output((s16)(rol_val_L2.out), (s16)(pit_val_L2.out),\
	rc_in.ch_processed[CH_THR], rc_in.ch_processed[CH_YAW]);
}
