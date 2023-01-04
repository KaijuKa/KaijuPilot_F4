#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"


#define gyro_deg2rad 0.01745f    //��ת����
#define gyro_para 0.061036f      //���ٶ� ϵ��
#define acc_para 0.002395f       //���ٶ� ϵ��
#define mag_para 0.146f          //����   ϵ��

#define acc_x_scale 0.999052f       //������еԭ���µĲ���ֵ����
#define acc_y_scale 0.999221f       //������еԭ���µĲ���ֵ����
#define acc_z_scale 1.001126f       //������еԭ���µĲ���ֵ����

#define ki_use 0.01f     //(0.001f + 0.0005f*(float)temp_1/500.0f)
#define kp_use 0.2f       //(0.3f + 0.1f*(float)temp_2/500.0f)
#define filter_k 0.22f    //(0.3f + 0.3f*(float)temp_3/500.0f)

//mpu���ݽṹ��
typedef struct{
	s16 gyro_x_raw;       //���ٶ� ϵ����0.061    �����ٶ� ��/s = 0.061*gyro_X
	s16 gyro_y_raw;
	s16 gyro_z_raw;
	
	s16 acc_x_raw;        //���ٶ� ϵ����0.00061
	s16 acc_y_raw;
	s16 acc_z_raw;
	
	s16 mag_x_raw;        //������ ϵ����0.146
	s16 mag_y_raw;
	s16 mag_z_raw;
	
	float gyro_x_err;   //���ٶ����
	float gyro_y_err;
	float gyro_z_err;
	
	float acc_x_err;    //���ٶ����
	float acc_y_err;
	float acc_z_err;
	
	s16 temp;         //�¶�
} MPU_Data_structure;

//imu�н��ٶȵ�λΪrad/s
//imu��̬����ṹ��
typedef struct{
	float w;            //4Ԫ��
	float x;
	float y;
	float z;
	
	float gyro_x;       //���ٶ� ϵ����0.061*3.14/180    �����ٶ� rad/s = gyro_X*0.061*3.14/180
	float gyro_y;
	float gyro_z;
	
	float acc_x;        //���ٶ� ϵ����0.00061 m/s2
	float acc_y;
	float acc_z;
	
	float mag_x;        //������ ϵ����0.146
	float mag_y;
	float mag_z;
	                    //���嵽����ı任���� a->w
	float att_matrix[3][3];
	
	float pit;
	float rol;
	float yaw;
	
} IMU_Data_structure;

extern IMU_Data_structure imu_data;
extern QueueHandle_t imu_data_queue;
	
u8 MPU_GYRO_Calibration(void);
void MPU_ACC_Calibration(void);
void IMU_Data_Update(void);
void IMU_Calcu(float dT_s);
void IMU_RPY_Calcu(void);
void IMU_Reset(void);
void IMU_Init(void);
void IMU_Data_Share(void);
#endif
