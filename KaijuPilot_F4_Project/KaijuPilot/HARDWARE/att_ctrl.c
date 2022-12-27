#include "att_ctrl.h"
#include "imu.h"
#include "arg_manage.h"

PID_ARG_structure rol_arg_L1 = {
	.kp = 250.0f,
	.kd = 4.0f,
	.ki = 0.0f,
	.expect_kd = 2.0f
};

PID_ARG_structure pit_arg_L1 = {
	.kp = 250.0f,
	.kd = 4.0f,
	.ki = 0.0f,
	.expect_kd = 2.0f
};

PID_VAL_structure rol_val_L1;
PID_VAL_structure pit_val_L1;

PID_ARG_structure rol_arg_L2 = {
	.kp = 7.0f,
	.kd = 0.07f,
	.ki = 0.7f,
	.expect_kd = 0.035f

};
PID_ARG_structure pit_arg_L2 = {
	.kp = 5.0f,
	.kd = 0.05f,
	.ki = 0.5f,
	.expect_kd = 0.025f

};

PID_VAL_structure rol_val_L2;
PID_VAL_structure pit_val_L2;


/*******************************************************************************
* 函 数 名         : att_val_init
* 函数功能		     : 姿态控制结构体初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void att_val_init(void)
{
	rol_val_L2.expect_old = 0;
	rol_val_L2.err_old = 0;
	rol_val_L2.out = 0;
	rol_val_L2.err_i = 0;
	
	pit_val_L2.expect_old = 0;
	pit_val_L2.err_old = 0;
	pit_val_L2.out = 0;
	pit_val_L2.err_i = 0;
}

/*******************************************************************************
* 函 数 名         : att_ctrl
* 函数功能		     : 姿态控制
* 输    入         : 周期 s 期望rol角度 期望pit角度 积分使能
* 输    出         : 无
*******************************************************************************/
void att_ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en)
{
	float rol_angular_spd_max_f = flight_arg.rol_angular_spd_max*gyro_para*gyro_deg2rad;
	float pit_angular_spd_max_f = flight_arg.pit_angular_spd_max*gyro_para*gyro_deg2rad;
	
	//外环 level2
	PID_calcu(dT_s, expect_rol, imu_data.rol, &rol_arg_L2, &rol_val_L2, 200, inter_en);
	PID_calcu(dT_s, expect_pit, imu_data.pit, &pit_arg_L2, &pit_val_L2, 200, inter_en);
	
	//限制大小
	rol_val_L1.out = LIMIT(rol_val_L2.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L2.out, -500, 500);
	
//	//限制大小
//	rol_val_L2.out = LIMIT(rol_val_L2.out, -rol_angular_spd_max_f, rol_angular_spd_max_f);
//	pit_val_L2.out = LIMIT(pit_val_L2.out, -pit_angular_spd_max_f, pit_angular_spd_max_f);
	
//	//内环 level1
//	PID_calcu(dT_s, rol_val_L2.out, imu_data.gyro_x, &rol_arg_L1, &rol_val_L1, 50);
//	PID_calcu(dT_s, pit_val_L2.out, imu_data.gyro_y, &pit_arg_L1, &pit_val_L1, 50);
//	
//	//限制大小
//	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
//	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}
