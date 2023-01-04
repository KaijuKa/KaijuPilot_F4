#include "par_manage.h"
#include "at24c02.h"

PAR_Store_union storaged_par;

PAR_INFO_structure par_info_array[PAR_NUM] = {
	{.par_max = 9000, .par_min = -9000, .name = "pit_offset         ", .info = "���ø���ƫ�� ��*100"},
	{.par_max = 9000, .par_min = -9000, .name = "rol_offset         ", .info = "���ú��ƫ�� ��*100"},
	{.par_max = 1, .par_min = 0, .name = "ch1_direct         ", .info = "����PWM1����       "},
	{.par_max = 1, .par_min = 0, .name = "ch2_direct         ", .info = "����PWM2����       "},
	{.par_max = 1, .par_min = 0, .name = "ch4_direct         ", .info = "����PWM4����       "},
	{.par_max = 100, .par_min = -100, .name = "ch1_offset         ", .info = "����PWM1ƫ��       "},
	{.par_max = 100, .par_min = -100, .name = "ch2_offset         ", .info = "����PWM2ƫ��       "},
	{.par_max = 100, .par_min = -100, .name = "ch3_offset         ", .info = "����PWM3ƫ��       "},
	{.par_max = 100, .par_min = -100, .name = "ch4_offset         ", .info = "����PWM4ƫ��       "},
	{.par_max = 50, .par_min = 1, .name = "ratio_att_rol      ", .info = "���ú���ж� 50Ϊ1 "},
	{.par_max = 50, .par_min = 1, .name = "ratio_att_pit      ", .info = "���ø����ж� 50Ϊ1 "},
	{.par_max = 50, .par_min = 1, .name = "ratio_rotation_rol ", .info = "����sp����ж�     "},
	{.par_max = 50, .par_min = 1, .name = "ratio_rotation_rol ", .info = "����sp�����ж�     "},
	{.par_max = 1, .par_min = 0, .name = "ctrl_mode          ", .info = "1��ͳ���� 0����    "},
	{.par_max = 90, .par_min = 0, .name = "rol_angle_max      ", .info = "������� ��      "},
	{.par_max = 90, .par_min = 0, .name = "pit_angle_max      ", .info = "������� ��      "},
	{.par_max = 300, .par_min = 0, .name = "rol_angular_spd_max", .info = "��������ٶ� ��/s"},
	{.par_max = 300, .par_min = 0, .name = "pit_angular_spd_max", .info = "��������ٶ� ��/s"},
	{.par_max = 50, .par_min = 10, .name = "target_air_speed   ", .info = "Ŀ�����m/s        "},
	{.par_max = 100, .par_min = 10, .name = "target_autoft_alt  ", .info = "Ŀ��߶�m          "},
	{.par_max = 100, .par_min = 0, .name = "auto_thr_max       ", .info = "����Զ�����%      "},
	{.par_max = 100, .par_min = 0, .name = "auto_thr_min       ", .info = "��С�Զ�����%      "},
	{.par_max = 90, .par_min = 0, .name = "auto_pit_max       ", .info = "������� ��      "}
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
* �� �� ��         : PAR_Load
* ��������		     : ��AT24�ж�������б�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void PAR_Load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//�������
	DRV_AT24_Read_Str(storaged_par.raw_data, LOAD_MAX, 0);
	
	//У��
	for(i = 0; i < LOAD_MAX-1; i++)
	{
		tmp += storaged_par.raw_data[i];
	}
	
	if(tmp == storaged_par.raw_data[LOAD_MAX-1])
	{
		//��ֵ
		fl_par.par = storaged_par.par;
	}
}

/*******************************************************************************
* �� �� ��         : PAR_Store
* ��������		     : �������в�����AT24
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void PAR_Store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_par.par = fl_par.par;
	
	//����У��
	for(i = 0; i < LOAD_MAX-1; i++)
	{
		tmp += storaged_par.raw_data[i];
	}
	storaged_par.raw_data[LOAD_MAX-1] = tmp;
	
	//д��AT24
	DRV_AT24_Write_Str(storaged_par.raw_data, LOAD_MAX, 0);
}

/*******************************************************************************
* �� �� ��         : PAR_Change
* ��������		     : �޸Ĳ���
* ��    ��         : ����id, ����ֵ
* ��    ��         : ��
*******************************************************************************/
void PAR_Change(u8 par_id, float par_val)
{
	static u8 cnt = 0;
	
	//����������в������б���
	if(cnt < PAR_NUM)
	{
		cnt ++;
	}
	else
	{
		cnt = 0;
		is_start_store = 1;
	}
	
	//����û�б仯 �˳�
	if(fl_par.s16_unit_array[par_id] == par_val)
	{
		return;
	}
	else 
	{
		//���������Ϲ涨 �˳�
		if(par_val < par_info_array[par_id].par_min || par_val > par_info_array[par_id].par_max)
		{
			return;
		}
		else
		{
			//�޸Ĳ�����������޸�
			fl_par.s16_unit_array[par_id] = (s16)par_val;
			is_par_dirty = 1;
		}
	}
}

/*******************************************************************************
* �� �� ��         : PAR_Store_Task
* ��������		     : ������������
* ��    ��         : dT_ms
* ��    ��         : ��
*******************************************************************************/
void PAR_Store_Task(u8 dT_ms)
{
	static u16 time_cnt;
	//���Կ�ʼ����
	if(is_start_store == 1)
	{
		//û�в����޸� ��λ��־
		if(is_par_dirty == 0)
		{
			is_start_store = 0;
		}
		//�в����޸� ���沢��λ��־
		else
		{
			//�ȴ�һ��ʱ��
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


