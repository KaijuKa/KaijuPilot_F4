#ifndef __MSG_INTERCHANGE_H__
#define __MSG_INTERCHANGE_H__

#include "stm32f4xx.h"

#define FRAME_HEAD 	   0xAB                //帧头
#define BOARDCAST_ADDR 0xFF                //广播地址
#define MCU_ADDR       0x11                //mcu地址

#define BYTE0(VAR)  (*((u8 *)(&VAR)    ))  //第一个字节
#define BYTE1(VAR)  (*((u8 *)(&VAR) + 1))  //第二个字节
#define BYTE2(VAR)  (*((u8 *)(&VAR) + 2))  //第三个字节
#define BYTE3(VAR)  (*((u8 *)(&VAR) + 3))  //第四个字节

//定义消息控制结构体
typedef struct{
	u8 wait2send;     //是否等待发送
	u8 dt_ms;         //周期
	u8 time_cnt_ms;   //已计时的时间
	u16 msg_info;     //消息控制附加信息
} MSG_Ctrl_structure;

//方便遍历
enum
{
	MSG_ID_MPU ,   //MPU信息
	MSG_ID_IMU ,   //imu欧拉角信息
	MSG_ID_HIGHT , //高度信息
	MSG_ID_TARIMU ,//目标imu信息
	MSG_ID_PWM ,   //pwm输出值信息
	MSG_ID_GPS ,   //gps位置信息
	MSG_ID_RC ,    //遥控信息
	MSG_ID_DEF,    //自定义数据帧
	DT_MSG_NUM,//7
};

//方便遍历
enum
{
	MSG_ID_PARNUM , //参数数量信息
	MSG_ID_PAR ,    //参数值信息
	MSG_ID_PARINFO ,//参数信息
	MSG_ID_DEV ,    //设备信息
	MSG_ID_ACK ,    //只对参数写入应答信息
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
