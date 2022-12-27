#include "flight_ctrl.h"
#include "remote_signal.h"
#include "pwm.h"
#include "arg_manage.h"
#include "imu.h"
#include "att_ctrl.h"

FLIGHT_CTRL_ST fl_st;

/*******************************************************************************
* �� �� ��         : flight_ctrl_task
* ��������		     : ���п����ܹ�����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void flight_ctrl_task(void)
{
	//����ͨ��5�õ���ǰģʽ
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
	
	//��ppm�ź�Ĭ�Ϸ���
	if(rc_in.ppm_offline > 1)
	{
		fl_st.flight_mode = Manual;
	}
	
	//������ ��ʾ��� ��ʼ����
	if(rc_in.ch_processed[CH_THR] > -400)
	{
		fl_st.flight_stat = 1;
	}
	else
	{
		fl_st.flight_stat = 0;
	}
	
	//���ݷ���ģʽִ����Ӧ����
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
* �� �� ��         : Manual_task
* ��������		     : �ֶ�ģʽ�µ��������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Manual_task(void)
{
	pwm_output(rc_in.ch_processed[CH_ROL], rc_in.ch_processed[CH_PIT],\
	rc_in.ch_processed[CH_THR], rc_in.ch_processed[CH_YAW]);
}

/*******************************************************************************
* �� �� ��         : Stabilize_task
* ��������		     : ����ģʽ�µ��������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Stabilize_task(void)
{
	float expect_rol = (rc_in.ch_processed[CH_ROL]/500.0f)*flight_arg.rol_angle_max;
	float expect_pit = (rc_in.ch_processed[CH_PIT]/500.0f)*flight_arg.pit_angle_max;
	
	att_ctrl(0.02f, expect_rol, expect_pit, fl_st.flight_stat);
	
	pwm_output((s16)(rol_val_L2.out), (s16)(pit_val_L2.out),\
	rc_in.ch_processed[CH_THR], rc_in.ch_processed[CH_YAW]);
}
