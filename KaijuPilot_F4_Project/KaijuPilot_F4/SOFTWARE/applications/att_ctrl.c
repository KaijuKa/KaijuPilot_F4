#include "att_ctrl.h"
#include "imu.h"
#include "arg_manage.h"

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
* �� �� ��         : ATT_VAL_Init
* ��������		     : ��̬���ƽṹ���ʼ��
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : ATT_Ctrl
* ��������		     : ��̬����
* ��    ��         : ���� s ����rol�Ƕ� ����pit�Ƕ� ����ʹ��
* ��    ��         : ��
*******************************************************************************/
void ATT_Ctrl(float dT_s, float expect_rol, float expect_pit, u8 inter_en)
{
	IMU_Data_structure imu_data;
	//����Ϣ������ȡ��imu����
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);
	
	//�⻷ level2
	pid_calcu(dT_s, expect_rol, imu_data.rol, &rol_arg_L2, &rol_val_L2, 200, inter_en);
	pid_calcu(dT_s, expect_pit, imu_data.pit, &pit_arg_L2, &pit_val_L2, 200, inter_en);
	
	//���ƴ�С
	rol_val_L2.out = LIMIT(rol_val_L2.out, -500, 500);
	pit_val_L2.out = LIMIT(pit_val_L2.out, -500, 500);
	
	//�л�Ϊrad/s��λ 0.6fΪ��СL2�����
	rol_val_L2.out *= 0.6f*gyro_deg2rad;
	pit_val_L2.out *= 0.6f*gyro_deg2rad;
	
	//�ڻ� level1
	pid_calcu(dT_s, rol_val_L2.out, imu_data.gyro_y, &rol_arg_L1, &rol_val_L1, 1, inter_en);
	pid_calcu(dT_s, pit_val_L2.out, imu_data.gyro_x, &pit_arg_L1, &pit_val_L1, 1, inter_en);
	
	//���ƴ�С 114.6f�Ŵ�pwmֵ
	rol_val_L1.out = rol_val_L1.out*114.6f*flight_arg.ratio_att_rol/50;
	pit_val_L1.out = pit_val_L1.out*114.6f*flight_arg.ratio_att_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}

/*******************************************************************************
* �� �� ��         : Rotation_Ctrl
* ��������		     : ��ת����
* ��    ��         : ���� s ����rol��ת�ٶ� ����pit��ת�ٶ� ����ʹ��
* ��    ��         : ��
*******************************************************************************/
void Rotation_Ctrl(float dT_s, float expect_rol_spd, float expect_pit_spd, u8 inter_en)
{
	IMU_Data_structure imu_data;
	//����Ϣ������ȡ��imu����
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);		
	
	//�л�Ϊrad/s��λ 0.8fΪ��С����
	expect_rol_spd *= 0.8f*gyro_deg2rad;
	expect_pit_spd *= 0.8f*gyro_deg2rad;
	
	//�ڻ� level1
	pid_calcu(dT_s, expect_rol_spd, imu_data.gyro_y, &rol_arg_L1, &rol_val_L1, 1, inter_en);
	pid_calcu(dT_s, expect_pit_spd, imu_data.gyro_x, &pit_arg_L1, &pit_val_L1, 1, inter_en);
	
	//���ƴ�С 229.2f�Ŵ�pwmֵ
	rol_val_L1.out = rol_val_L1.out*229.2f*flight_arg.ratio_rotation_rol/50;
	pit_val_L1.out = pit_val_L1.out*229.2f*flight_arg.ratio_rotation_pit/50;
	rol_val_L1.out = LIMIT(rol_val_L1.out, -500, 500);
	pit_val_L1.out = LIMIT(pit_val_L1.out, -500, 500);
}
