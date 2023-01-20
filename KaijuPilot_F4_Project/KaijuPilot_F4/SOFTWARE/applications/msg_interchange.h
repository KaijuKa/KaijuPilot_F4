#ifndef __MSG_INTERCHANGE_H__
#define __MSG_INTERCHANGE_H__

#include "stm32f4xx.h"

#define FRAME_HEAD 	   0xAB                //֡ͷ
#define BOARDCAST_ADDR 0xFF                //�㲥��ַ
#define MCU_ADDR       0x11                //mcu��ַ

#define BYTE0(VAR)  (*((u8 *)(&VAR)    ))  //��һ���ֽ�
#define BYTE1(VAR)  (*((u8 *)(&VAR) + 1))  //�ڶ����ֽ�
#define BYTE2(VAR)  (*((u8 *)(&VAR) + 2))  //�������ֽ�
#define BYTE3(VAR)  (*((u8 *)(&VAR) + 3))  //���ĸ��ֽ�

//������Ϣ���ƽṹ��
typedef struct{
	u8 wait2send;     //�Ƿ�ȴ�����
	u8 dt_ms;         //����
	u8 time_cnt_ms;   //�Ѽ�ʱ��ʱ��
	u16 msg_info;     //��Ϣ���Ƹ�����Ϣ
} MSG_Ctrl_structure;

//�������
enum
{
	MSG_ID_MPU ,   //MPU��Ϣ
	MSG_ID_IMU ,   //imuŷ������Ϣ
	MSG_ID_HIGHT , //�߶���Ϣ
	MSG_ID_TARIMU ,//Ŀ��imu��Ϣ
	MSG_ID_PWM ,   //pwm���ֵ��Ϣ
	MSG_ID_GPS ,   //gpsλ����Ϣ
	MSG_ID_RC ,    //ң����Ϣ
	MSG_ID_DEF,    //�Զ�������֡
	DT_MSG_NUM,//7
};

//�������
enum
{
	MSG_ID_PARNUM , //����������Ϣ
	MSG_ID_PAR ,    //����ֵ��Ϣ
	MSG_ID_PARINFO ,//������Ϣ
	MSG_ID_DEV ,    //�豸��Ϣ
	MSG_ID_ACK ,    //ֻ�Բ���д��Ӧ����Ϣ
	TG_MSG_NUM,//5
};

void DT_MSG_Init(void);
void MSG_Ctrl_Task(u8 dT_ms);
void DT_MSG_Frame_Send(u8 fun_id);
void TG_MSG_Frame_Send(u8 fun_id);
void MSG_Check_ADD(u8 *buf, u8 len);
void MSG_RECV_ByteGet(u8 data);
void MSG_RECV_Analysis(u8 len);
void MSG_Send(void);

#endif
