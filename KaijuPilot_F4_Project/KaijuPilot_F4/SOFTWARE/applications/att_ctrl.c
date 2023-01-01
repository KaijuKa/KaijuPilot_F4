#include "att_ctrl.h"
#include "imu.h"
#include "arg_manage.h"
#include "kaiju_math.h"

#include "FreeRTOS.h"
#include "queue.h"

PID_ARG_structure rol_arg_L1 = {
	.kp = 0.8f,
	.kd = 0.012f,
	.ki = 0.0f, //last 0.0f
	.expect_kd = 0.0f,
	.fb_kd = 0.015f
};

PID_ARG_structure pit_arg_L1 = {
	.kp = 0.8f,
	.kd = 0.012f,
	.ki = 0.0f, //last 0.0f
	.expect_kd = 0.0f,
	.fb_kd = 0.015f
};

PID_VAL_structure rol_val_L1;
PID_VAL_structure pit_val_L1;

PID_ARG_structure rol_arg_L2 = {
	.kp = 7.0f,
	.kd = 0.07f,
	.ki = 0.5f,
	.expect_kd = 0.028f,
	.fb_kd = 0.0f

};

PID_ARG_structure pit_arg_L2 = {
	.kp = 5.0f,
	.kd = 0.05f,
	.ki = 0.35f,
	.expect_kd = 0.02f,
	.fb_kd = 0.0f
};

PID_VAL_structure rol_val_L2;
PID_VAL_structure pit_val_L2;


/*******************************************************************************
* 函 数 名         : ATT_VAL_Init
* 函数功能		     : 姿态控制结构体初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ATT_VAL_Init(void)
{
	rol_val_L2.expect_old = 0;
	rol_val_L2.err_old = 0;
	rol_val_L2.out = 0;
	rol_val_L2.err_i = 0;
	rol_val_L2.fb_old = 0;
	
	pit_val_L2.expect_old = 0;
	pit_val_L2.err_old = 0;
	pit_val_L2.out = 0;
	pit_val_L2.err_i = 0;
	pit_val_L2.fb_old = 0;
	
	rol_val_L1.expect_old = 0;
	rol_val_L1.err_old = 0;
	rol_val_L1.out = 0;
	rol_val_L1.err_i = 0;
	rol_val_L1.fb_old = 0;
	
	pit_val_L1.expect_old = 0;
	pit_val_L1.err_old = 0;
	pit_val_L1.out = 0;
	pit_val_L1.err_i = 0;
	pit_val_L1.fb_old = 0;
}

/*******************************************************************************
* 函 数 名         : ATT_Ctrl
* 函数功能		     : 姿态控制
* 输    入         : 周期 s 期望rol角度 期望pit角度 积分使能
* 输    出         : 无
*******************************************************************************/
void ATT_Ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en)
{
	IMU_Data_structure imu_data;
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);
	
	//外环 level2
	pid_calcu(dT_s, expect_rol, imu_data.rol, &rol_arg_L2, &rol_val_L2, 200, inter_en);
	pid_calcu(dT_s, expect_pit, imu_data.pit, &pit_arg_L2, &pit_val_L2, 200, inter_en);
	
	//限制大小
	rol_val_L2.out = LIMIT(rol_val_L2.out, -500, 500);
	pit_val_L2.out = LIMIT(pit_val_L2.out, -500, 500);
	
	//切换为rad/s单位 0.6f为缩小L2环输出
	rol_val_L2.out *= 0.6f*gyro_deg2rad;
	pit_val_L2.out *= 0.6f*gyro_deg2rad;
	
	//内环 level1
	pid_calcu(dT_s, rol_val_L2.out, imu_data.gyro_y, &rol_arg_L1, &rol_val_L1, 1, inter_en);
	pid_calcu(dT_s, pit_val_L2.out, imu_data.gyro_x, &pit_arg_L1, &pit_val_L1, 1, inter_en);
	
	//限制大小 114.6f放大到pwm值
	rol_val_L1.out = rol_val_L1.out*114.6f*flight_arg.ratio_att_rol/50;
	pit_val_L1.out = pit_val_L1.out*114.6f*flight_arg.ratio_att_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}

/*******************************************************************************
* 函 数 名         : Rotation_Ctrl
* 函数功能		     : 滚转控制
* 输    入         : 周期 s 期望rol滚转速度 期望pit滚转速度 积分使能
* 输    出         : 无
*******************************************************************************/
void Rotation_Ctrl(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en)
{
	IMU_Data_structure imu_data;
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);		
	
	//切换为rad/s单位 0.8f为缩小输入
	expect_rol_spd *= 0.8f*gyro_deg2rad;
	expect_pit_spd *= 0.8f*gyro_deg2rad;
	
	//内环 level1
	pid_calcu(dT_s, expect_rol_spd, imu_data.gyro_y, &rol_arg_L1, &rol_val_L1, 1, inter_en);
	pid_calcu(dT_s, expect_pit_spd, imu_data.gyro_x, &pit_arg_L1, &pit_val_L1, 1, inter_en);
	
	//限制大小 229.2f放大到pwm值
	rol_val_L1.out = rol_val_L1.out*229.2f*flight_arg.ratio_rotation_rol/50;
	pit_val_L1.out = pit_val_L1.out*229.2f*flight_arg.ratio_rotation_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}

/*******************************************************************************
* 函 数 名         : Rotation_ctrl2
* 函数功能		     : 滚转控制
* 输    入         : 周期 s 期望rol滚转速度 期望pit滚转速度 积分使能
* 输    出         : 无
*******************************************************************************/
void Rotation_ctrl2(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en)
{	
	static float expect_rol_spd_lpf = 0;
	static float expect_pit_spd_lpf = 0;
	static float w,x,y,z;            //总期望四元数
	float w_tmp, x_tmp, y_tmp, z_tmp;//临时四元数
	float pnqn[4][4] = {0};          //四元数计算中间结果
	float expect_rol_err_set;        //期望角度误差设定值
	float expect_pit_err_set;        
	float expect_rol_err;            //期望角度误差经反馈处理后
	float expect_pit_err;
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3;
	float t_temp;
	
	IMU_Data_structure imu_data;
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);	
	
	//速度增量限幅
	expect_rol_spd_lpf += LIMIT((expect_rol_spd - expect_rol_spd_lpf), -30, 30);
	expect_pit_spd_lpf += LIMIT((expect_pit_spd - expect_pit_spd_lpf), -30, 30);
	
	//计算此次期望角度误差
	expect_rol_err_set = expect_rol_spd_lpf*dT_s;
	expect_pit_err_set = expect_pit_spd_lpf*dT_s;
	
	expect_rol_err_set = expect_rol_err_set/2;
	expect_pit_err_set = expect_pit_err_set/2;
	
	//计算四元数, q(err_set) = q(rol)*q(pit)
	//计算中间结果 跳过了两个四元数的构建
	//pit对应x轴旋转 rol对应y轴旋转
	//pit的四元数：my_cos(expect_pit_err_set), my_sin(expect_pit_err_set), 0, 0
	//rol的四元数：my_cos(expect_rol_err_set), 0, my_sin(expect_rol_err_set), 0
	pnqn[0][0] = my_cos(expect_rol_err_set)*my_cos(expect_pit_err_set);
	pnqn[0][1] = my_cos(expect_rol_err_set)*my_sin(expect_pit_err_set);
	pnqn[2][0] = my_sin(expect_rol_err_set)*my_cos(expect_pit_err_set);
	pnqn[2][1] = my_sin(expect_rol_err_set)*my_sin(expect_pit_err_set);
	
	//q(err_set)
	w_tmp = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x_tmp = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y_tmp = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z_tmp = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	//计算四元数, q(new) = q(err_set)*q(old)
	//计算中间结果
	pnqn[0][0] = w_tmp*w;
	pnqn[0][1] = w_tmp*x;
	pnqn[0][2] = w_tmp*y;
	pnqn[0][3] = w_tmp*z;
	pnqn[1][0] = x_tmp*w;
	pnqn[1][1] = x_tmp*x;
	pnqn[1][2] = x_tmp*y;
	pnqn[1][3] = x_tmp*z;
	pnqn[2][0] = y_tmp*w;
	pnqn[2][1] = y_tmp*x;
	pnqn[2][2] = y_tmp*y;
	pnqn[2][3] = y_tmp*z;
	pnqn[3][0] = z_tmp*w;
	pnqn[3][1] = z_tmp*x;
	pnqn[3][2] = z_tmp*y;
	pnqn[3][3] = z_tmp*z;
	
	//q(new)
	w = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	//取当前姿态的共轭
	imu_data.x = -imu_data.x;
	imu_data.y = -imu_data.y;
	imu_data.z = -imu_data.z;
	
	//计算四元数, q(err) = q(new)*(q(fb)-1)
	//计算中间结果
	pnqn[0][0] = w*imu_data.w;
	pnqn[0][1] = w*imu_data.x;
	pnqn[0][2] = w*imu_data.y;
	pnqn[0][3] = w*imu_data.z;
	pnqn[1][0] = x*imu_data.w;
	pnqn[1][1] = x*imu_data.x;
	pnqn[1][2] = x*imu_data.y;
	pnqn[1][3] = x*imu_data.z;
	pnqn[2][0] = y*imu_data.w;
	pnqn[2][1] = y*imu_data.x;
	pnqn[2][2] = y*imu_data.y;
	pnqn[2][3] = y*imu_data.z;
	pnqn[3][0] = z*imu_data.w;
	pnqn[3][1] = z*imu_data.x;
	pnqn[3][2] = z*imu_data.y;
	pnqn[3][3] = z*imu_data.z;
	
	//q(err)
	w_tmp = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x_tmp = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y_tmp = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z_tmp = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	q0q1 = w_tmp * x_tmp;
	q0q2 = w_tmp * y_tmp;
	q1q1 = x_tmp * x_tmp;
	q1q3 = x_tmp * z_tmp;
	q2q2 = y_tmp * y_tmp;
	q2q3 = y_tmp * z_tmp;
	
	t_temp = LIMIT(1 - my_pow(2*q1q3 - 2*q0q2),0,1);
	
	expect_pit_err = fast_atan2(2*q2q3 + 2*q0q1, 1- (2*q1q1 + 2*q2q2))*57.30f;
	expect_rol_err = -fast_atan2(2*q1q3 - 2*q0q2, my_sqrt(t_temp))*57.30f;
	
		//外环 level2
	pid_calcu(dT_s, expect_rol_err, 0, &rol_arg_L2, &rol_val_L2, 200, inter_en);
	pid_calcu(dT_s, expect_pit_err, 0, &pit_arg_L2, &pit_val_L2, 200, inter_en);
	
	//限制大小
	rol_val_L2.out = LIMIT(rol_val_L2.out, -500, 500);
	pit_val_L2.out = LIMIT(pit_val_L2.out, -500, 500);
	
	//切换为rad/s单位 0.6f为缩小L2环输出
	rol_val_L2.out *= 0.6f*gyro_deg2rad;
	pit_val_L2.out *= 0.6f*gyro_deg2rad;
	
	//内环 level1
	pid_calcu(dT_s, rol_val_L2.out, imu_data.gyro_y, &rol_arg_L1, &rol_val_L1, 1, inter_en);
	pid_calcu(dT_s, pit_val_L2.out, imu_data.gyro_x, &pit_arg_L1, &pit_val_L1, 1, inter_en);
	
	//限制大小 114.6f放大到pwm值
	rol_val_L1.out = rol_val_L1.out*114.6f*flight_arg.ratio_att_rol/50;
	pit_val_L1.out = pit_val_L1.out*114.6f*flight_arg.ratio_att_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}
