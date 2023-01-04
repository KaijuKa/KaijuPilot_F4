#include "imu.h"
#include "uart.h"
#include "delay.h"
#include "icm20602.h"
#include "kaiju_math.h"
#include "par_manage.h"

#include "FreeRTOS.h"
#include "queue.h"



/* 
	俯视图机头
	pit
	icm20602 y
	|
	|
	|
	|
	|________ rol icm20602 x
	
	因此
	姿态环pit对应gyro_x
	姿态环rol对应gyro_y
	
	姿态解算中的机体坐标是icm20602的xyz坐标
gyro遵守右手定则, 拇指指向坐标轴正方向 四指方向为角速度正方向
*/


//经过matlab椭球拟合出的acc x,y,z 
static MPU_Data_structure mpu_data = {0,0,0,0,0,0,0,0,0,0,0,0,-34.8312f,89.6591f,42.8492f,0};
IMU_Data_structure imu_data = {1,0,0,0,
							0,0,0,
							0,0,0,
							0,0,0,
							{{0,0,0},
							{0,0,0},
							{0,0,0}},
							0,0,0
						};

QueueHandle_t imu_data_queue = NULL;

/*******************************************************************************
* 函 数 名         : MPU_GYRO_Calibration
* 函数功能		     : MPU校准
* 输    入         : 无
* 输    出         : 1 失败 0 成功
*******************************************************************************/
u8 MPU_GYRO_Calibration(void)
{
	s32 gyro_x_sum = 0;
	s32 gyro_y_sum = 0;
	s32 gyro_z_sum = 0;
	u16 times;
	u8 mpu_buffer[14];
	
	//清零误差
	mpu_data.gyro_x_err = 0;
	mpu_data.gyro_y_err = 0;
	mpu_data.gyro_z_err = 0;
	
	for(times = 0;times < 200;times++)
	{
		//获取数据
		DRV_Icm20602_Read(mpu_buffer);
	
		//读取buffer原始数据
		mpu_data.gyro_x_raw = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
		mpu_data.gyro_y_raw = -(s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
		mpu_data.gyro_z_raw = -(s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;
		
		//求和
		gyro_x_sum += mpu_data.gyro_x_raw;
		gyro_y_sum += mpu_data.gyro_y_raw;
		gyro_z_sum += mpu_data.gyro_z_raw;
		
		delay_ms(2);
	}
	
	//计算误差
	mpu_data.gyro_x_err = gyro_x_sum/200.0f;
	mpu_data.gyro_y_err = gyro_y_sum/200.0f;
	mpu_data.gyro_z_err = gyro_z_sum/200.0f;
	
	return 0;
}

/*******************************************************************************
* 函 数 名         : MPU_ACC_Calibration
* 函数功能		     : 校准acc
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void MPU_ACC_Calibration(void)
{
	u8 i = 0;
	u8 mpu_buffer[14];
	
	if(Rx_data == 0xA5) //打印标识
	{
		for(i = 0;i < 50;i++)
		{
			//获取acc数据
			DRV_Icm20602_Read(mpu_buffer);
	
			//读取buffer原始数据
			mpu_data.acc_x_raw = (s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);
			mpu_data.acc_y_raw = -(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);
			mpu_data.acc_z_raw = -(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);
			
			printf("%d %d %d\n",mpu_data.acc_x_raw, mpu_data.acc_y_raw, mpu_data.acc_z_raw);
			delay_ms(20);
		}
		Rx_data = 0x00;
	}
}

/*******************************************************************************
* 函 数 名         : IMU_Data_Update
* 函数功能		     : imu初始数据更新
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IMU_Data_Update(void)
{
	static float gyro_lpf[5][3] = {0};
	static float acc_lpf[5][3] = {0};
	
	u8 mpu_buffer[14];
	DRV_Icm20602_Read(mpu_buffer);
	
	//读取buffer原始数据
	mpu_data.acc_x_raw = (s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);
	mpu_data.acc_y_raw = -(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);
	mpu_data.acc_z_raw = -(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);
 
	mpu_data.gyro_x_raw = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
	mpu_data.gyro_y_raw = -(s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
	mpu_data.gyro_z_raw = -(s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;
	
	mpu_data.temp = (s16)((((s16)mpu_buffer[6]) << 8) | mpu_buffer[7]);
	
	
	//去漂移 和 缩放
	gyro_lpf[4][0] = mpu_data.gyro_x_raw-mpu_data.gyro_x_err;
	gyro_lpf[4][1] = mpu_data.gyro_y_raw-mpu_data.gyro_y_err;
	gyro_lpf[4][2] = mpu_data.gyro_z_raw-mpu_data.gyro_z_err;
	
	acc_lpf[4][0] = (mpu_data.acc_x_raw-mpu_data.acc_x_err)*acc_x_scale;
	acc_lpf[4][1] = (mpu_data.acc_y_raw-mpu_data.acc_y_err)*acc_y_scale;
	acc_lpf[4][2] = (mpu_data.acc_z_raw-mpu_data.acc_z_err)*acc_z_scale;
	
	//4阶低通滤波
	for(s8 i = 3;i >= 0;i--)
	{
		gyro_lpf[i][0] = gyro_lpf[i][0]*(1.0f - filter_k) + gyro_lpf[i+1][0]*filter_k;
		gyro_lpf[i][1] = gyro_lpf[i][1]*(1.0f - filter_k) + gyro_lpf[i+1][1]*filter_k;
		gyro_lpf[i][2] = gyro_lpf[i][2]*(1.0f - filter_k) + gyro_lpf[i+1][2]*filter_k;
		
		acc_lpf[i][0] = acc_lpf[i][0]*(1.0f - filter_k) + acc_lpf[i+1][0]*filter_k;
		acc_lpf[i][1] = acc_lpf[i][1]*(1.0f - filter_k) + acc_lpf[i+1][1]*filter_k;
		acc_lpf[i][2] = acc_lpf[i][2]*(1.0f - filter_k) + acc_lpf[i+1][2]*filter_k;
	}
	
	//转换成 rad/s 和 m/s2
	imu_data.gyro_x = (float)gyro_lpf[0][0]*gyro_para*gyro_deg2rad;
	imu_data.gyro_y = (float)gyro_lpf[0][1]*gyro_para*gyro_deg2rad;
	imu_data.gyro_z = (float)gyro_lpf[0][2]*gyro_para*gyro_deg2rad;
	 
	imu_data.acc_x = (float)acc_lpf[0][0]*acc_para;
	imu_data.acc_y = (float)acc_lpf[0][1]*acc_para;
	imu_data.acc_z = (float)acc_lpf[0][2]*acc_para;
}

/*******************************************************************************
* 函 数 名         : IMU_Calcu
* 函数功能		     : imu姿态解算
* 输    入         : 周期(s)
* 输    出         : 无
*******************************************************************************/
//机体坐标下的z方向向量 等效重力向量
float z_vec[3];
float vec_err_i[3] = {0,0,0};
void IMU_Calcu(float dT_s)
{
	int i;
	
	//计算acc模的倒数
	float acc_norm_l_recip = my_sqrt_reciprocal(imu_data.acc_x*imu_data.acc_x \
	                                   +imu_data.acc_y*imu_data.acc_y \
	                                   +imu_data.acc_z*imu_data.acc_z);
	
	
	//计算acc模
	float acc_norm_l = safe_div(1,acc_norm_l_recip,0);
	//q模倒数
	float q_norm_l_recip;
	
	float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;
	
	//单位化的acc
	float acc_norm[3];
	//叉乘误差											
	float vec_err[3];
	//微分角度
	float d_angle[3];
    //误差积分
	
	acc_norm[0] = acc_norm_l_recip*imu_data.acc_x;
	acc_norm[1] = acc_norm_l_recip*imu_data.acc_y;
	acc_norm[2] = acc_norm_l_recip*imu_data.acc_z;
	
	q0q1 = imu_data.w * imu_data.x;
	q0q2 = imu_data.w * imu_data.y;
	q1q1 = imu_data.x * imu_data.x;
	q1q3 = imu_data.x * imu_data.z;
	q2q2 = imu_data.y * imu_data.y;
	q2q3 = imu_data.y * imu_data.z;
	q3q3 = imu_data.z * imu_data.z;
	q1q2 = imu_data.x * imu_data.y;
	q0q3 = imu_data.w * imu_data.z;
	
	//计算转换矩阵
	imu_data.att_matrix[0][0] = 1 - (2*q2q2 + 2*q3q3);
	imu_data.att_matrix[0][1] = 2*q1q2 - 2*q0q3;
	imu_data.att_matrix[0][2] = 2*q1q3 + 2*q0q2;
		
	imu_data.att_matrix[1][0] = 2*q1q2 + 2*q0q3;
	imu_data.att_matrix[1][1] = 1 - (2*q1q1 + 2*q3q3);
	imu_data.att_matrix[1][2] = 2*q2q3 - 2*q0q1;
	
	//机体坐标下的z方向向量 等效重力向量
	imu_data.att_matrix[2][0] = z_vec[0] = 2*q1q3 - 2*q0q2;
	imu_data.att_matrix[2][1] = z_vec[1] = 2*q2q3 + 2*q0q1;
	imu_data.att_matrix[2][2] = z_vec[2] = 1 - (2*q1q1 + 2*q2q2);
	
	
	
	//误差计算
	vec_err[0] =  (acc_norm[1] * z_vec[2] - z_vec[1] * acc_norm[2]);
	vec_err[1] = -(acc_norm[0] * z_vec[2] - z_vec[0] * acc_norm[2]);
	vec_err[2] = -(acc_norm[1] * z_vec[0] - z_vec[1] * acc_norm[0]);
	
	//误差积分计算
	for(i = 0;i<3;i++)
	{
		if(acc_norm_l>10.6f || acc_norm_l<9.0f)
		{
			vec_err[0] = vec_err[1] = vec_err[2] = 0;
		}
		vec_err_i[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) *dT_s *ki_use;
	}

	//计算微分角度
	d_angle[0] = (imu_data.gyro_x + (vec_err[0]  + vec_err_i[0]) * kp_use ) * dT_s / 2 ;
	d_angle[1] = (imu_data.gyro_y + (vec_err[1]  + vec_err_i[1]) * kp_use ) * dT_s / 2 ;
	d_angle[2] = (imu_data.gyro_z + (vec_err[2]  + vec_err_i[2]) * kp_use ) * dT_s / 2 ;
	
	//计算4元数
	imu_data.w = imu_data.w            - imu_data.x*d_angle[0] - imu_data.y*d_angle[1] - imu_data.z*d_angle[2];
	imu_data.x = imu_data.w*d_angle[0] + imu_data.x            + imu_data.y*d_angle[2] - imu_data.z*d_angle[1];
	imu_data.y = imu_data.w*d_angle[1] - imu_data.x*d_angle[2] + imu_data.y            + imu_data.z*d_angle[0];
	imu_data.z = imu_data.w*d_angle[2] + imu_data.x*d_angle[1] - imu_data.y*d_angle[0] + imu_data.z;
	
	//单位化
	q_norm_l_recip = my_sqrt_reciprocal(imu_data.w*imu_data.w + imu_data.x*imu_data.x +imu_data.y*imu_data.y + imu_data.z*imu_data.z);

	imu_data.w *= q_norm_l_recip;
	imu_data.x *= q_norm_l_recip;
	imu_data.y *= q_norm_l_recip;
	imu_data.z *= q_norm_l_recip;
}

/*******************************************************************************
* 函 数 名         : IMU_RPY_Calcu
* 函数功能		     : 计算pit rol yaw
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
static float t_temp;
void IMU_RPY_Calcu(void)
{
	t_temp = LIMIT(1 - my_pow(imu_data.att_matrix[2][0]),0,1);
	
	
	if(z_vec[2]>0.05f || z_vec[2]<-0.05f)
	{
		imu_data.pit =  fast_atan2(imu_data.att_matrix[2][1], imu_data.att_matrix[2][2])*57.30f - fl_par.par.pit_offset/100.0f;
		imu_data.rol =   -fast_atan2(imu_data.att_matrix[2][0], my_sqrt(t_temp))*57.30f - fl_par.par.rol_offset/100.0f;
		imu_data.yaw =  fast_atan2(imu_data.att_matrix[1][0], imu_data.att_matrix[0][0])*57.30f; 
	}
}

/*******************************************************************************
* 函 数 名         : IMU_Reset
* 函数功能		     : 复位imu
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IMU_Reset(void)
{
	u8 i;
	u8 j;
	
	imu_data.w = 1;
	imu_data.x = 0;
	imu_data.y = 0;
	imu_data.z = 0;
	
	imu_data.acc_x = 0;
	imu_data.acc_y = 0;
	imu_data.acc_z = 0;
	
	for(i = 0;i<3;i++)
	{
		for(j = 0;j<3;j++)
		{
			imu_data.att_matrix[i][j] = 0;
		}
	}
}

/*******************************************************************************
* 函 数 名         : IMU_Init
* 函数功能		     : imu初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IMU_Init(void)
{
	//角速度校准
	IMU_Reset();
	MPU_GYRO_Calibration();
	delay_ms(1000);
}

/*******************************************************************************
* 函 数 名         : IMU_Data_Share
* 函数功能		     : imu数据写入消息队列供分享
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IMU_Data_Share(void)
{
	static u8 isFullQueue = 0;
	
	if(0 == isFullQueue)
	{
		isFullQueue = 1;
		//创建消息队列并写入
		imu_data_queue = xQueueCreate(1, sizeof(imu_data));
		xQueueSend(imu_data_queue, (void *)&imu_data, 0);
	}
	else
	{
		//消息队列覆写
		xQueueOverwrite(imu_data_queue, (void *)&imu_data);
	}
}
