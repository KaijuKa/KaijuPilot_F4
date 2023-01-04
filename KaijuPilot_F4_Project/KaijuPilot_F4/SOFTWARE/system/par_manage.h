#ifndef __PAR_MANAGE_H__
#define __PAR_MANAGE_H__

//#include "io.h"
#include "stm32f4xx.h"

#define STORE_DELAY 500       //��ʼ�������ǰ����ʱ �������ݴ���У�����

#define LOAD_MAX 64           //�����б���eeprom��Ĵ�С

#define PAR_NUM 23            //��������

#define PAR_DESC_FRAME_LEN 50 //������Ϣ֡��С

#define PAR_NAME_LEN 20       //�������ֳ���
 
#define PAR_INFO_LEN 20       //������Ϣ����

//���в����ṹ��
typedef struct{
	s16 pit_offset; //imu pit rolˮƽУ׼ƫ�� ��λ��*100 0-9000
	s16 rol_offset;
	
	s16 ch1_direct;    //ͨ��124 ������� 1/0
	s16 ch2_direct;
	s16 ch4_direct;
	
	s16 ch1_offset;   //ͨ��1234�����ƫ�� ֱ�Ӳ���pwm��� -100-100
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	s16 ratio_att_rol;     //pit rol ���ȵ�ģʽ�ж� 50Ϊ1
	s16 ratio_att_pit; 
	
	s16 ratio_rotation_rol; //pit rol �˶�ģʽ�ж� 50Ϊ1
	s16 ratio_rotation_pit;
	
	s16 ctrl_mode;     //�̶���ģʽ 0������ 1��ͳ����
	
	s16 rol_angle_max; //����ģʽ�� �����б�Ƕ� ��λ�� 0-90
	s16 pit_angle_max;                                  
	
					  //���ֶ�ģʽ�� �����ת�ٶ� ��λ��/s
	s16 rol_angular_spd_max;
	s16 pit_angular_spd_max;
	
					  //�Զ�Ѳ��ģʽ�� �����趨ֵ �� ���θ߶��趨ֵ
	s16 target_air_speed;      //��λ m/s  0-127
	s16 target_autoft_altitude;//��λ m    0-127
	s16 auto_thr_max;          //�������  0-100
	s16 auto_thr_min;          //��С����  0-100
	s16 auto_pit_max;          //pit����ֵ 0-90
	
} PAR_structure;

//������Ϣ�ṹ��
typedef struct{
	s16 par_max;           //�������ֵ
	s16 par_min;           //������Сֵ
	u8 name[PAR_NAME_LEN]; //�������� �ַ���
	u8 info[PAR_INFO_LEN]; //������Ϣ �ַ���
} PAR_INFO_structure;

//���ڲ����ı���
typedef union{
	PAR_structure par;
	u8 raw_data[LOAD_MAX];
} PAR_Store_union;

//���ڲ������޸ĺͶ���
typedef union{
	PAR_structure par;
	s16 s16_unit_array[LOAD_MAX/2];
} PAR_MSG_union;

extern PAR_MSG_union fl_par;
extern PAR_Store_union storaged_par;
extern PAR_INFO_structure par_info_array[PAR_NUM];

void PAR_Load(void);
void PAR_Store(void);

void PAR_Change(u8 par_id, float par_val);
void PAR_Store_Task(u8 dT_ms);


#endif
