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
* 函 数 名         : arg_load
* 函数功能		     : 从AT24中读入参数列表
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void arg_load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//读入参数
	AT24_Read_Str(storaged_arg.raw_data, 40, 0);
	
	//校验
	for(i = 0; i < 32; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	
	if(tmp == storaged_arg.raw_data[32])
	{
		//赋值
		flight_arg = storaged_arg.arg;
	}
}

/*******************************************************************************
* 函 数 名         : arg_store
* 函数功能		     : 加载所有参数到AT24
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void arg_store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_arg.arg = flight_arg;
	
	//计算校验
	for(i = 0; i < 32; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	storaged_arg.raw_data[32] = tmp;
	
	//写入AT24
	AT24_Write_Str(storaged_arg.raw_data, 40, 0);
}

/*******************************************************************************
* 函 数 名         : argSet_byteGet
* 函数功能		     : 单次更新参数
* 输    入         : 接收到的字节
* 输    出         : 无
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
			//更新参数
		}
		state = 0;
	}
	else
	{
		buf[state-3] = data;
		state++;
	}
}
