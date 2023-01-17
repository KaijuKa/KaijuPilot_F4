#include "att_ctrl.h"
#include "imu.h"
#include "par_manage.h"
#include "kaiju_math.h"

#include "FreeRTOS.h"
#include "queue.h"

Rotation_structure rotation_data = {1, 0, 0, 0, 0, 0};

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
	IMU_Data_structure imu_data;
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);
	
	rol_val_L2.expect_old = 0;
	rol_val_L2.err_old = 0;
	rol_val_L2.out = 0;
	rol_val_L2.fb_old = 0;
	
	pit_val_L2.expect_old = 0;
	pit_val_L2.err_old = 0;
	pit_val_L2.out = 0;
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
	
	rotation_data.w = imu_data.w;
	rotation_data.x = imu_data.x;
	rotation_data.y = imu_data.y;
	rotation_data.z = imu_data.z;
	
	rotation_data.expect_rol_spd_lpf = 0;
	rotation_data.expect_pit_spd_lpf = 0;
	
	rotation_data.expect_pit_err = 0;
	rotation_data.expect_rol_err = 0;
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
	rol_val_L1.out = rol_val_L1.out*114.6f*fl_par.par.ratio_att_rol/50;
	pit_val_L1.out = pit_val_L1.out*114.6f*fl_par.par.ratio_att_pit/50;
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
	rol_val_L1.out = rol_val_L1.out*229.2f*fl_par.par.ratio_rotation_rol/50;
	pit_val_L1.out = pit_val_L1.out*229.2f*fl_par.par.ratio_rotation_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}

/*******************************************************************************
* 函 数 名         : Rotation_Ctrl2
* 函数功能		     : 滚转控制
* 输    入         : 周期 s 期望rol滚转速度 期望pit滚转速度 积分使能
* 输    出         : 无
*******************************************************************************/
#define ONE_PI   (3.14159265f)
void Rotation_Ctrl2(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en)
{	
	u8 i;
	float w_tmp, x_tmp, y_tmp, z_tmp;//临时四元数
	float w,x,y,z;                   //期望四元数去掉yaw
	float pnqn[4][4] = {0};          //四元数计算中间结果
	float expect_rol_err_set;        //期望角度误差设定值
	float expect_pit_err_set;        
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q1q2,q0q3,q3q3;
	float t_temp;
	float z_vec_old_w[3];              //机体坐标z轴在世界坐标系下的表示
	float z_vec_new_w[3];              //期望机体坐标z轴在世界坐标系下的表示
	float z_vec_new_a[3];              //期望机体坐标z轴在机体坐标系下的表示
	float vec_err[3];                  //新旧z轴坐标的叉积
	float vec_err_norm_l_recip; 
	float vec1_norm_l, vec2_norm_l;    //两个向量的平方根
	float sin_vec_err,cos_vec_err;     //新旧z轴的夹角三角函数
	float yaw_err;                     //期望四元数与反馈四元数的yaw差
	
	IMU_Data_structure imu_data;
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);	
	
	//取上次期望姿态的共轭
	//q(ex_last)
	rotation_data.w_last = rotation_data.w;
	rotation_data.x_last = rotation_data.x;
	rotation_data.y_last = rotation_data.y;
	rotation_data.z_last = rotation_data.z;
	
	rotation_data.x_last = -rotation_data.x_last;
	rotation_data.y_last = -rotation_data.y_last;
	rotation_data.z_last = -rotation_data.z_last;
	
	//计算四元数, q(err_fb) = q(fb)*(q(ex_last)-1)
	//计算中间结果
	pnqn[0][0] = imu_data.w*rotation_data.w_last;
	pnqn[0][1] = imu_data.w*rotation_data.x_last;
	pnqn[0][2] = imu_data.w*rotation_data.y_last;
	pnqn[0][3] = imu_data.w*rotation_data.z_last;
	pnqn[1][0] = imu_data.x*rotation_data.w_last;
	pnqn[1][1] = imu_data.x*rotation_data.x_last;
	pnqn[1][2] = imu_data.x*rotation_data.y_last;
	pnqn[1][3] = imu_data.x*rotation_data.z_last;
	pnqn[2][0] = imu_data.y*rotation_data.w_last;
	pnqn[2][1] = imu_data.y*rotation_data.x_last;
	pnqn[2][2] = imu_data.y*rotation_data.y_last;
	pnqn[2][3] = imu_data.y*rotation_data.z_last;
	pnqn[3][0] = imu_data.z*rotation_data.w_last;
	pnqn[3][1] = imu_data.z*rotation_data.x_last;
	pnqn[3][2] = imu_data.z*rotation_data.y_last;
	pnqn[3][3] = imu_data.z*rotation_data.z_last;
	
	//q(err_fb, 相对于世界坐标系)
	w_tmp = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x_tmp = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y_tmp = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z_tmp = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	q2q2 = y_tmp*y_tmp;
	q3q3 = z_tmp*z_tmp;
	q1q2 = x_tmp*y_tmp;
	q0q3 = w_tmp*z_tmp;
	
	//计算yaw偏差
	yaw_err = fast_atan2(2*q1q2 + 2*q0q3, 1 - (2*q2q2 + 2*q3q3))*57.30f; 
	yaw_err /= 2;
	yaw_err = yaw_err*ONE_PI/180.0f;
	
	//分解出绕Z轴的四元数
	w_tmp = my_cos(yaw_err);
	x_tmp = 0;
	y_tmp = 0;
	z_tmp = my_sin(yaw_err);
	
	//计算四元数, q(ex_last_fixyaw) = q(err_fb)*q(ex_last)
	//计算中间结果
	pnqn[0][0] = w_tmp*rotation_data.w;
	pnqn[0][1] = w_tmp*rotation_data.x;
	pnqn[0][2] = w_tmp*rotation_data.y;
	pnqn[0][3] = w_tmp*rotation_data.z;
	pnqn[1][0] = x_tmp*rotation_data.w;
	pnqn[1][1] = x_tmp*rotation_data.x;
	pnqn[1][2] = x_tmp*rotation_data.y;
	pnqn[1][3] = x_tmp*rotation_data.z;
	pnqn[2][0] = y_tmp*rotation_data.w;
	pnqn[2][1] = y_tmp*rotation_data.x;
	pnqn[2][2] = y_tmp*rotation_data.y;
	pnqn[2][3] = y_tmp*rotation_data.z;
	pnqn[3][0] = z_tmp*rotation_data.w;
	pnqn[3][1] = z_tmp*rotation_data.x;
	pnqn[3][2] = z_tmp*rotation_data.y;
	pnqn[3][3] = z_tmp*rotation_data.z;
	
	//q(ex_last_fixyaw)
	rotation_data.w = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	rotation_data.x = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	rotation_data.y = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	rotation_data.z = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	//速度增量限幅
	rotation_data.expect_rol_spd_lpf += LIMIT((expect_rol_spd - rotation_data.expect_rol_spd_lpf), -30, 30);
	rotation_data.expect_pit_spd_lpf += LIMIT((-expect_pit_spd - rotation_data.expect_pit_spd_lpf), -30, 30);
	
	//计算此次期望角度误差
	expect_rol_err_set = rotation_data.expect_rol_spd_lpf*dT_s;
	expect_pit_err_set = rotation_data.expect_pit_spd_lpf*dT_s;
	
	expect_rol_err_set = expect_rol_err_set/2;
	expect_pit_err_set = expect_pit_err_set/2;
	
	expect_rol_err_set = expect_rol_err_set*ONE_PI/180.0f;
	expect_pit_err_set = expect_pit_err_set*ONE_PI/180.0f;
	
	//计算四元数, q(err_set, 相较于机体坐标系) = q(rol)*q(pit)
	//计算中间结果 跳过了两个四元数的构建
	//pit对应x轴旋转 rol对应y轴旋转
	//pit的四元数：my_cos(expect_pit_err_set), my_sin(expect_pit_err_set), 0, 0
	//rol的四元数：my_cos(expect_rol_err_set), 0, my_sin(expect_rol_err_set), 0
	pnqn[0][0] = my_cos(expect_rol_err_set)*my_cos(expect_pit_err_set);
	pnqn[0][1] = my_cos(expect_rol_err_set)*my_sin(expect_pit_err_set);
	pnqn[0][2] = 0;
	pnqn[0][3] = 0;
	pnqn[1][0] = 0;
	pnqn[1][1] = 0;
	pnqn[1][2] = 0;
	pnqn[1][3] = 0;
	pnqn[2][0] = my_sin(expect_rol_err_set)*my_cos(expect_pit_err_set);
	pnqn[2][1] = my_sin(expect_rol_err_set)*my_sin(expect_pit_err_set);
	pnqn[2][2] = 0;
	pnqn[2][3] = 0;
	pnqn[3][0] = 0;
	pnqn[3][1] = 0;
	pnqn[3][2] = 0;
	pnqn[3][3] = 0;
	
	//q(err_set, 相较于机体坐标系)
	w_tmp = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x_tmp = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y_tmp = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z_tmp = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	q1q1 = x_tmp*x_tmp;
	q2q2 = y_tmp*y_tmp;
	q0q1 = w_tmp*x_tmp;
	q0q2 = w_tmp*y_tmp;
	q1q3 = x_tmp*z_tmp;
	q2q3 = y_tmp*z_tmp;
	
	//计算q(err_set, 相较于机体坐标系)的Z轴在机体坐标系下的表示
	z_vec_new_a[0] = 2*(q1q3 + q0q2);
	z_vec_new_a[1] = 2*(q2q3 + q0q1);
	z_vec_new_a[2] = 1-(2*q1q1 + 2*q2q2);
	
	//计算q(err_set, 相较于机体坐标系)的Z轴在世界坐标系下的表示
	for(i = 0; i < 3; i++)
	{
		z_vec_new_w[i] = z_vec_new_a[0]*imu_data.att_matrix[i][0] +
					   z_vec_new_a[1]*imu_data.att_matrix[i][1] +
					   z_vec_new_a[2]*imu_data.att_matrix[i][2];
	}
	
	//计算机体坐标系的z轴在世界坐标系下的表示
	z_vec_old_w[0] = imu_data.att_matrix[0][2];
	z_vec_old_w[1] = imu_data.att_matrix[1][2];
	z_vec_old_w[2] = imu_data.att_matrix[2][2];
	
	
	//叉积
	vec_err[0] = z_vec_old_w[1]*z_vec_new_w[2] - z_vec_old_w[2]*z_vec_new_w[1];
	vec_err[1] = z_vec_old_w[2]*z_vec_new_w[0] - z_vec_old_w[0]*z_vec_new_w[2];
	vec_err[2] = z_vec_old_w[0]*z_vec_new_w[1] - z_vec_old_w[1]*z_vec_new_w[0];
	
	//单位化
	vec_err_norm_l_recip = my_sqrt_reciprocal(vec_err[0]*vec_err[0] \
	                                   +vec_err[1]*vec_err[1]  \
	                                   +vec_err[2]*vec_err[2] );
	
	vec_err[0] *= vec_err_norm_l_recip;
	vec_err[1] *= vec_err_norm_l_recip;
	vec_err[2] *= vec_err_norm_l_recip;
	
	//构造四元数需要夹角/2 这里取中间向量直接计算 三角函数
	z_vec_old_w[0] += z_vec_new_w[0];
	z_vec_old_w[1] += z_vec_new_w[1];
	z_vec_old_w[2] += z_vec_new_w[2];
	
	z_vec_old_w[0] /= 2;
	z_vec_old_w[1] /= 2;
	z_vec_old_w[2] /= 2;
	
	vec1_norm_l = my_sqrt(z_vec_old_w[0]*z_vec_old_w[0] \
	                                   +z_vec_old_w[1]*z_vec_old_w[1]  \
	                                   +z_vec_old_w[2]*z_vec_old_w[2] );
									   
	vec2_norm_l = my_sqrt(z_vec_new_w[0]*z_vec_new_w[0] \
	                                   +z_vec_new_w[1]*z_vec_new_w[1]  \
	                                   +z_vec_new_w[2]*z_vec_new_w[2] );		   
	
	cos_vec_err = z_vec_old_w[0]*z_vec_new_w[0] + z_vec_old_w[1]*z_vec_new_w[1] + z_vec_old_w[2]*z_vec_new_w[2] /
					(vec1_norm_l*vec2_norm_l);
	
	sin_vec_err = my_sqrt(1 - cos_vec_err*cos_vec_err);
	
	//计算在世界坐标下的新旧机体坐标系的旋转四元数
	//q(err_set, 相较于世界坐标系)
	w_tmp = cos_vec_err;
	x_tmp = vec_err[0] * sin_vec_err;
	y_tmp = vec_err[1] * sin_vec_err;
	z_tmp = vec_err[2] * sin_vec_err;
	
	
	
	//计算四元数, q(ex_new) = q(err_set, 相较于世界坐标系)*q(ex_last_fixyaw)
	//计算中间结果
	pnqn[0][0] = w_tmp*rotation_data.w;
	pnqn[0][1] = w_tmp*rotation_data.x;
	pnqn[0][2] = w_tmp*rotation_data.y;
	pnqn[0][3] = w_tmp*rotation_data.z;
	pnqn[1][0] = x_tmp*rotation_data.w;
	pnqn[1][1] = x_tmp*rotation_data.x;
	pnqn[1][2] = x_tmp*rotation_data.y;
	pnqn[1][3] = x_tmp*rotation_data.z;
	pnqn[2][0] = y_tmp*rotation_data.w;
	pnqn[2][1] = y_tmp*rotation_data.x;
	pnqn[2][2] = y_tmp*rotation_data.y;
	pnqn[2][3] = y_tmp*rotation_data.z;
	pnqn[3][0] = z_tmp*rotation_data.w;
	pnqn[3][1] = z_tmp*rotation_data.x;
	pnqn[3][2] = z_tmp*rotation_data.y;
	pnqn[3][3] = z_tmp*rotation_data.z;
	
	//q(ex_new)
	rotation_data.w = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	rotation_data.x = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	rotation_data.y = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	rotation_data.z = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	//分解出绕Z轴的四元数
	w_tmp = my_cos(-imu_data.yaw*ONE_PI/360.0f);
	x_tmp = 0;
	y_tmp = 0;
	z_tmp = my_sin(-imu_data.yaw*ONE_PI/360.0f);
	
	pnqn[0][0] = w_tmp*rotation_data.w;
	pnqn[0][1] = w_tmp*rotation_data.x;
	pnqn[0][2] = w_tmp*rotation_data.y;
	pnqn[0][3] = w_tmp*rotation_data.z;
	pnqn[1][0] = x_tmp*rotation_data.w;
	pnqn[1][1] = x_tmp*rotation_data.x;
	pnqn[1][2] = x_tmp*rotation_data.y;
	pnqn[1][3] = x_tmp*rotation_data.z;
	pnqn[2][0] = y_tmp*rotation_data.w;
	pnqn[2][1] = y_tmp*rotation_data.x;
	pnqn[2][2] = y_tmp*rotation_data.y;
	pnqn[2][3] = y_tmp*rotation_data.z;
	pnqn[3][0] = z_tmp*rotation_data.w;
	pnqn[3][1] = z_tmp*rotation_data.x;
	pnqn[3][2] = z_tmp*rotation_data.y;
	pnqn[3][3] = z_tmp*rotation_data.z;
	
	//消除yaw分量的ex_new
	w = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	x = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	y = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	z = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	pnqn[0][0] = w_tmp*imu_data.w;
	pnqn[0][1] = w_tmp*imu_data.x;
	pnqn[0][2] = w_tmp*imu_data.y;
	pnqn[0][3] = w_tmp*imu_data.z;
	pnqn[1][0] = x_tmp*imu_data.w;
	pnqn[1][1] = x_tmp*imu_data.x;
	pnqn[1][2] = x_tmp*imu_data.y;
	pnqn[1][3] = x_tmp*imu_data.z;
	pnqn[2][0] = y_tmp*imu_data.w;
	pnqn[2][1] = y_tmp*imu_data.x;
	pnqn[2][2] = y_tmp*imu_data.y;
	pnqn[2][3] = y_tmp*imu_data.z;
	pnqn[3][0] = z_tmp*imu_data.w;
	pnqn[3][1] = z_tmp*imu_data.x;
	pnqn[3][2] = z_tmp*imu_data.y;
	pnqn[3][3] = z_tmp*imu_data.z;
	
	//消除yaw分量的fb
	imu_data.w = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	imu_data.x = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	imu_data.y = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	imu_data.z = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	//取fb的共轭
	imu_data.x = -imu_data.x;
	imu_data.y = -imu_data.y;
	imu_data.z = -imu_data.z;
	

	//计算四元数, q(err) = q(ex_new)*(q(fb)-1)
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
	rotation_data.w_err = pnqn[0][0] - pnqn[1][1] - pnqn[2][2] - pnqn[3][3];
	rotation_data.x_err = pnqn[1][0] + pnqn[0][1] + pnqn[2][3] - pnqn[3][2];
	rotation_data.y_err = pnqn[2][0] + pnqn[0][2] + pnqn[3][1] - pnqn[1][3];
	rotation_data.z_err = pnqn[3][0] + pnqn[0][3] + pnqn[1][2] - pnqn[2][1];
	
	q0q1 = rotation_data.w_err * rotation_data.x_err;
	q0q2 = rotation_data.w_err * rotation_data.y_err;
	q1q1 = rotation_data.x_err * rotation_data.x_err;
	q1q3 = rotation_data.x_err * rotation_data.z_err;
	q2q2 = rotation_data.y_err * rotation_data.y_err;
	q2q3 = rotation_data.y_err * rotation_data.z_err;
	
	
	//计算偏差角
	t_temp = LIMIT(1 - my_pow(2*q1q3 - 2*q0q2),0,1);
	
	rotation_data.expect_pit_err = fast_atan2(2*q2q3 + 2*q0q1, 1- (2*q1q1 + 2*q2q2))*57.30f;
	rotation_data.expect_rol_err = -fast_atan2(2*q1q3 - 2*q0q2, my_sqrt(t_temp))*57.30f;
	
	//外环 level2
	pid_calcu(dT_s, rotation_data.expect_rol_err, 0, &rol_arg_L2, &rol_val_L2, 200, inter_en);
	pid_calcu(dT_s, rotation_data.expect_pit_err, 0, &pit_arg_L2, &pit_val_L2, 200, inter_en);
	
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
	rol_val_L1.out = rol_val_L1.out*114.6f*fl_par.par.ratio_rotation_rol/50;
	pit_val_L1.out = pit_val_L1.out*114.6f*fl_par.par.ratio_rotation_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}

