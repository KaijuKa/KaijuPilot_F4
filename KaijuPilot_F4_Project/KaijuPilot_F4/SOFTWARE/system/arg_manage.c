#include "arg_manage.h"
#include "at24c02.h"

ARG_union storaged_arg;

ARG_structure flight_arg = {
	.pit_offset = 0,
	.rol_offset = 0,
	
	.ch1_direct = 1,
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
	
	.rol_angular_spd_max = 200,
	.pit_angular_spd_max = 150
};

/*******************************************************************************
* �� �� ��         : ARG_Load
* ��������		     : ��AT24�ж�������б�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ARG_Load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//�������
	DRV_AT24_Read_Str(storaged_arg.raw_data, LOAD_MAX, 0);
	
	//У��
	for(i = 0; i < LOAD_MAX-2; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	
	if(tmp == storaged_arg.raw_data[LOAD_MAX-1])
	{
		//��ֵ
		flight_arg = storaged_arg.arg;
	}
}

/*******************************************************************************
* �� �� ��         : arg_store
* ��������		     : �������в�����AT24
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ARG_Store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_arg.arg = flight_arg;
	
	//����У��
	for(i = 0; i < LOAD_MAX-2; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	storaged_arg.raw_data[LOAD_MAX-1] = tmp;
	
	//д��AT24
	DRV_AT24_Write_Str(storaged_arg.raw_data, LOAD_MAX, 0);
}
