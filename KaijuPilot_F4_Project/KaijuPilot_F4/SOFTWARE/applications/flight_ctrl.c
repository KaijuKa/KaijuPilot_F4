#include "flight_ctrl.h"
#include "remote_signal.h"
#include "pwm.h"
#include "arg_manage.h"
#include "imu.h"
#include "att_ctrl.h"

FLIGHT_Data_structure fl_data;

/*******************************************************************************
* 函 数 名         : Flight_Ctrl_Task
* 函数功能		     : 飞行控制总管任务
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void Flight_Ctrl_Task(u8 dT_ms)
{	
	static u8 last_flight_mode = 0;
	
	//根据通道5得到当前模式
	if(rc_data.ch_processed[AUX1] > 300)
	{
		fl_data.flight_mode = Manual;   //高位
	}
	else if (rc_data.ch_processed[AUX1] > -200)
	{
		fl_data.flight_mode = Stabilize;//中位
	}
	else
	{
		fl_data.flight_mode = Sport;    //低位
	}
	
	//切换模式 复位pid val
	if(last_flight_mode != fl_data.flight_mode)
	{
		ATT_VAL_Init();
		last_flight_mode = fl_data.flight_mode;
	}
	
#ifdef IS_SBUS
	//无sbus信号默认自稳
	if(rc_data.sbus_offline > 0)
	{
		fl_data.flight_mode = Stabilize;
	}
#else
	//无ppm信号默认自稳
	if(rc_data.ppm_offline > 1)
	{
		fl_data.flight_mode = Stabilize;
	}
#endif
	
	//有油门 表示起飞 开始积分
	if(rc_data.ch_processed[CH_THR] > -400)
	{
		fl_data.flight_stat = 1;
	}
	else
	{
		fl_data.flight_stat = 0;
	}
	
	//根据飞行模式执行相应操作
	switch(fl_data.flight_mode)
	{
		case Manual:
		{
			Manual_Task();
			break;
		}
		case Stabilize:
		{
			Stabilize_Task(dT_ms);
			break;
		}
		case Sport:
		{
			Sport_Task(dT_ms);
			break;
		}
		case RTL:
		{
			break;
		}
	}
}

/*******************************************************************************
* 函 数 名         : Manual_Task
* 函数功能		     : 手动模式下的外设控制
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void Manual_Task(void)
{
	DRV_PWM_Output(rc_data.ch_processed[CH_ROL], rc_data.ch_processed[CH_PIT],\
	rc_data.ch_processed[CH_THR], rc_data.ch_processed[CH_YAW]);
}

/*******************************************************************************
* 函 数 名         : Stabilize_Task
* 函数功能		     : 自稳模式下的外设控制
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void Stabilize_Task(u8 dT_ms)
{
	float expect_rol = (rc_data.ch_processed[CH_ROL]/500.0f)*flight_arg.rol_angle_max;
    float expect_pit = (rc_data.ch_processed[CH_PIT]/500.0f)*flight_arg.pit_angle_max;
	
	ATT_Ctrl(0.02f, expect_rol, expect_pit, fl_data.flight_stat);
	
	DRV_PWM_Output((s16)(rol_val_L1.out), (s16)(pit_val_L1.out),\
	rc_data.ch_processed[CH_THR], rc_data.ch_processed[CH_YAW]);
}

/*******************************************************************************
* 函 数 名         : Sport_Task
* 函数功能		     : 运动模式下的外设控制
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void Sport_Task(u8 dT_ms)
{
	//添加死区限制
	float expect_rol_spd = (my_deadzone(rc_data.ch_processed[CH_ROL], 0, 20)/500.0f)*flight_arg.rol_angular_spd_max;
    float expect_pit_spd = (my_deadzone(rc_data.ch_processed[CH_PIT], 0, 20)/500.0f)*flight_arg.pit_angular_spd_max;
	
	Rotation_Ctrl2(0.02f, expect_rol_spd, expect_pit_spd, fl_data.flight_stat);
	
	DRV_PWM_Output((s16)(rol_val_L1.out), (s16)(pit_val_L1.out),\
	rc_data.ch_processed[CH_THR], rc_data.ch_processed[CH_YAW]);
}

