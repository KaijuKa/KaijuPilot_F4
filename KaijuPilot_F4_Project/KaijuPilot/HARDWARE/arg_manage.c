#include "arg_manage.h"
#include "at24c02.h"

arg_union storaged_arg;

arg_st flight_arg = {
	.pit_offset = 0,
	.rol_offset = 0,
	
	.ch1_direct = 0,
	.ch2_direct = 1,
	.ch4_direct = 0,
	
	.ch1_offset = 0,
	.ch2_offset = 0,
	.ch3_offset = 0,
	.ch4_offset = 0,
	
	.ratio_rol = 50,
	.ratio_pit = 50,
	
	.ctrl_mode = 0,
	
	.rol_angle_max = 65,
	.pit_angle_max = 50,
	
	.rol_angular_spd_max = 600,
	.pit_angular_spd_max = 600
};

/*******************************************************************************
* �� �� ��         : arg_load
* ��������		     : ��AT24�ж�������б�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void arg_load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//�������
	AT24_Read_Str(storaged_arg.raw_data, 40, 0);
	
	//У��
	for(i = 0; i < 32; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	
	if(tmp == storaged_arg.raw_data[32])
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
void arg_store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_arg.arg = flight_arg;
	
	//����У��
	for(i = 0; i < 32; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	storaged_arg.raw_data[32] = tmp;
	
	//д��AT24
	AT24_Write_Str(storaged_arg.raw_data, 40, 0);
}

/*******************************************************************************
* �� �� ��         : argSet_byteGet
* ��������		     : ���θ��²���
* ��    ��         : ���յ����ֽ�
* ��    ��         : ��
*******************************************************************************/
void argSet_byteGet(u8 data)
{
	static u8 state = 0;
	static u8 target_arg = 0;
	static u8 buf[4];
	
	if(state == 0)
	{
		if(0x5a == data)
		{
			state++;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 1)
	{
		if(0xa5 == data)
		{
			state++;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 2)
	{
		target_arg = data;
		state++;
	}
	else if(state == 7)
	{
		if(0xff == data)
		{
			//���²���
		}
		state = 0;
	}
	else
	{
		buf[state-3] = data;
		state++;
	}
}
