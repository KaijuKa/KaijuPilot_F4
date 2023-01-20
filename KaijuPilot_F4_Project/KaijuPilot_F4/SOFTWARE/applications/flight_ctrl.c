#include "flight_ctrl.h"
#include "remote_signal.h"
#include "pwm.h"
#include "par_manage.h"
#include "imu.h"
#include "att_ctrl.h"
#include "pos_calcu.h"
#include "route_ctrl.h"

#include "FreeRTOS.h"
#include "queue.h"

FLIGHT_Data_structure fl_data;

QueueHandle_t fl_data_queue = NULL;

/*******************************************************************************
* �� �� ��         : Flight_Data_Share
* ��������		     : Flight����д����Ϣ���й�����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void Flight_Data_Share(void)
{
	static u8 isFullQueue = 0;
	
	if(0 == isFullQueue)
	{
		isFullQueue = 1;
		//������Ϣ���в�д��
		fl_data_queue = xQueueCreate(1, sizeof(fl_data));
		xQueueSend(fl_data_queue, (void *)&fl_data, 0);
	}
	else
	{
		//��Ϣ���и�д
		xQueueOverwrite(fl_data_queue, (void *)&fl_data);
	}
}

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
		if(rc_data.ch_processed[AUX2] > 300)
		{
			fl_data.flight_mode = Sport;    //��λ
		}
		else if(rc_data.ch_processed[AUX2] > -200)
		{
			fl_data.flight_mode = RTL;    //��λ
		}
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
	
	//ȷ���ҵ�λ��
	if(fl_data.pos_stat == 0)
	{
		if(pos_data.fix_sta > 0 && pos_data.star_num > 6)
		{
			//�߶ȳ�ʼ��
			RAW_Height_Calibration();
			
			fl_data.pos_stat = 1;
			fl_data.pos_log = pos_data.log;
			fl_data.pos_lat = pos_data.lat;
		}
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
			RTL_Task(dT_ms);
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
	fl_data.pwm_out[CH_ROL] = rc_data.ch_processed[CH_ROL];
	fl_data.pwm_out[CH_PIT] = rc_data.ch_processed[CH_PIT];
	fl_data.pwm_out[CH_THR] = rc_data.ch_processed[CH_THR];
	fl_data.pwm_out[CH_YAW] = rc_data.ch_processed[CH_YAW];
	
	DRV_PWM_Output(fl_data.pwm_out[CH_ROL], fl_data.pwm_out[CH_PIT],\
	fl_data.pwm_out[CH_THR], fl_data.pwm_out[CH_YAW]);
}

/*******************************************************************************
* �� �� ��         : Stabilize_Task
* ��������		     : ����ģʽ�µ��������
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void Stabilize_Task(u8 dT_ms)
{
	float expect_rol = (rc_data.ch_processed[CH_ROL]/500.0f)*fl_par.par.rol_angle_max;
    float expect_pit = (rc_data.ch_processed[CH_PIT]/500.0f)*fl_par.par.pit_angle_max;
	
	fl_data.target_rol = expect_rol;
	fl_data.target_pit = expect_pit;
	
	ATT_Ctrl(dT_ms/1000.0f, expect_rol, expect_pit, fl_data.flight_stat);
	
	fl_data.pwm_out[CH_ROL] = (s16)(rol_val_L1.out);
	fl_data.pwm_out[CH_PIT] = (s16)(pit_val_L1.out);
	fl_data.pwm_out[CH_THR] = rc_data.ch_processed[CH_THR];
	fl_data.pwm_out[CH_YAW] = rc_data.ch_processed[CH_YAW];
	
	DRV_PWM_Output(fl_data.pwm_out[CH_ROL], fl_data.pwm_out[CH_PIT],\
	fl_data.pwm_out[CH_THR], fl_data.pwm_out[CH_YAW]);
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
	float expect_rol_spd = (my_deadzone(rc_data.ch_processed[CH_ROL], 0, 20)/500.0f)*fl_par.par.rol_angular_spd_max;
    float expect_pit_spd = (my_deadzone(rc_data.ch_processed[CH_PIT], 0, 20)/500.0f)*fl_par.par.pit_angular_spd_max;
	
	fl_data.target_rol = expect_rol_spd;
	fl_data.target_pit = expect_pit_spd;
	
	Rotation_Ctrl2(dT_ms/1000.0f, expect_rol_spd, expect_pit_spd, fl_data.flight_stat);
	
	fl_data.pwm_out[CH_ROL] = (s16)(rol_val_L1.out);
	fl_data.pwm_out[CH_PIT] = (s16)(pit_val_L1.out);
	fl_data.pwm_out[CH_THR] = rc_data.ch_processed[CH_THR];
	fl_data.pwm_out[CH_YAW] = rc_data.ch_processed[CH_YAW];
	
	DRV_PWM_Output(fl_data.pwm_out[CH_ROL], fl_data.pwm_out[CH_PIT],\
	fl_data.pwm_out[CH_THR], fl_data.pwm_out[CH_YAW]);
}

/*******************************************************************************
* �� �� ��         : RTL_Task
* ��������		     : �˶�ģʽ�µ��������
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void RTL_Task(u8 dT_ms)
{
	float expect_pit = (rc_data.ch_processed[CH_PIT]/500.0f)*fl_par.par.pit_angle_max;
	float expect_rol = 0;
	
	if(1 == fl_data.pos_stat)
	{
		expect_rol = Route_Ctrl(dT_ms, fl_data.pos_log, fl_data.pos_lat);
	}
	
	fl_data.target_rol = expect_rol;
	fl_data.target_pit = expect_pit;
	
	ATT_Ctrl(dT_ms/1000.0f, expect_rol, expect_pit, fl_data.flight_stat);
	
	fl_data.pwm_out[CH_ROL] = (s16)(rol_val_L1.out);
	fl_data.pwm_out[CH_PIT] = (s16)(pit_val_L1.out);
	fl_data.pwm_out[CH_THR] = rc_data.ch_processed[CH_THR];
	fl_data.pwm_out[CH_YAW] = rc_data.ch_processed[CH_YAW];
	
	DRV_PWM_Output(fl_data.pwm_out[CH_ROL], fl_data.pwm_out[CH_PIT],\
	fl_data.pwm_out[CH_THR], fl_data.pwm_out[CH_YAW]);
}

