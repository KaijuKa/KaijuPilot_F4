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
* 函 数 名         : ARG_Load
* 函数功能		     : 从AT24中读入参数列表
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ARG_Load(void)
{
	u8 tmp = 0;
	u8 i;
	
	//读入参数
	DRV_AT24_Read_Str(storaged_arg.raw_data, LOAD_MAX, 0);
	
	//校验
	for(i = 0; i < LOAD_MAX-2; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	
	if(tmp == storaged_arg.raw_data[LOAD_MAX-1])
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
void ARG_Store(void)
{
	u8 tmp = 0;
	u8 i;
	
	storaged_arg.arg = flight_arg;
	
	//计算校验
	for(i = 0; i < LOAD_MAX-2; i++)
	{
		tmp += storaged_arg.raw_data[i];
	}
	storaged_arg.raw_data[LOAD_MAX-1] = tmp;
	
	//写入AT24
	DRV_AT24_Write_Str(storaged_arg.raw_data, LOAD_MAX, 0);
}
