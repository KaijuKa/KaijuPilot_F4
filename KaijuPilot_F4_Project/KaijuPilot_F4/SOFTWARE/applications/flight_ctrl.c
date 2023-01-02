#include "flight_ctrl.h"
#include "remote_signal.h"
#include "pwm.h"
#include "arg_manage.h"
#include "imu.h"
#include "att_ctrl.h"

FLIGHT_Data_structure fl_data;

/*******************************************************************************
* �� �� ��         : Flight_Ctrl_Task
* ��������		     : ���п����ܹ�����
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void Flight_Ctrl_Task(u8 dT_ms)
{	
	static u8 last_flight_mode = 0;
	
	//����ͨ��5�õ���ǰģʽ
	if(rc_data.ch_processed[AUX1] > 300)
	{
		fl_data.flight_mode = Manual;   //��λ
	}
	else if (rc_data.ch_processed[AUX1] > -200)
	{
		fl_data.flight_mode = Stabilize;//��λ
	}
	else
	{
		fl_data.flight_mode = Sport;    //��λ
	}
	
	//�л�ģʽ ��λpid val
	if(last_flight_mode != fl_data.flight_mode)
	{
		ATT_VAL_Init();
		last_flight_mode = fl_data.flight_mode;
	}
	
#ifdef IS_SBUS
	//��sbus�ź�Ĭ������
	if(rc_data.sbus_offline > 0)
	{
		fl_data.flight_mode = Stabilize;
	}
#else
	//��ppm�ź�Ĭ������
	if(rc_data.ppm_offline > 1)
	{
		fl_data.flight_mode = Stabilize;
	}
#endif
	
	//������ ��ʾ��� ��ʼ����
	if(rc_data.ch_processed[CH_THR] > -400)
	{
		fl_data.flight_stat = 1;
	}
	else
	{
		fl_data.flight_stat = 0;
	}
	
	//���ݷ���ģʽִ����Ӧ����
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
* �� �� ��         : Manual_Task
* ��������		     : �ֶ�ģʽ�µ��������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Manual_Task(void)
{
	DRV_PWM_Output(rc_data.ch_processed[CH_ROL], rc_data.ch_processed[CH_PIT],\
	rc_data.ch_processed[CH_THR], rc_data.ch_processed[CH_YAW]);
}

/*******************************************************************************
* �� �� ��         : Stabilize_Task
* ��������		     : ����ģʽ�µ��������
* ��    ��         : ����ms
* ��    ��         : ��
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
* �� �� ��         : Sport_Task
* ��������		     : �˶�ģʽ�µ��������
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void Sport_Task(u8 dT_ms)
{
	//�����������
	float expect_rol_spd = (my_deadzone(rc_data.ch_processed[CH_ROL], 0, 20)/500.0f)*flight_arg.rol_angular_spd_max;
    float expect_pit_spd = (my_deadzone(rc_data.ch_processed[CH_PIT], 0, 20)/500.0f)*flight_arg.pit_angular_spd_max;
	
	Rotation_Ctrl2(0.02f, expect_rol_spd, expect_pit_spd, fl_data.flight_stat);
	
	DRV_PWM_Output((s16)(rol_val_L1.out), (s16)(pit_val_L1.out),\
	rc_data.ch_processed[CH_THR], rc_data.ch_processed[CH_YAW]);
}

