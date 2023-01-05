#include "msg_interchange.h"
#include "imu.h"
#include "uart.h"
#include "remote_signal.h"
#include "att_ctrl.h"
#include "flight_ctrl.h"
#include "par_manage.h"

#include "FreeRTOS.h"
#include "queue.h"

//周期性消息帧数组
MSG_Ctrl_structure dt_msg_array[DT_MSG_NUM];
//触发性消息帧数组
MSG_Ctrl_structure tg_msg_array[TG_MSG_NUM];

//枚举对应到真实的帧ID
u8 DT_MSG_ID_ARRAY[DT_MSG_NUM] = {0x01, 0x03, 0x04, 0x05, 0x07, 0x20, 0x30, 0x40};

u8 TG_MSG_ID_ARRAY[TG_MSG_NUM] = {0xE0, 0xE1, 0xE2, 0xE3, 0x00};

/*******************************************************************************
* 函 数 名         : DT_MSG_Init
* 函数功能		     : 周期性信息发送控制结构体初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DT_MSG_Init(void)
{
	//惯性传感器数据
	dt_msg_array[MSG_ID_MPU].dt_ms = 20;
	
	//欧拉角
	dt_msg_array[MSG_ID_IMU].dt_ms = 20;
	
	//四元数
	dt_msg_array[MSG_ID_IMU2].dt_ms = 20;
	
	//高度
	dt_msg_array[MSG_ID_HIGHT].dt_ms = 200;
	
	//飞行速度
	dt_msg_array[MSG_ID_SPD].dt_ms = 200;
	
	//pwm输出
	dt_msg_array[MSG_ID_PWM].dt_ms = 20;
	
	//GPS位置
	dt_msg_array[MSG_ID_GPS].dt_ms = 200;
	
	//遥控器
	dt_msg_array[MSG_ID_RC].dt_ms = 20;
}

/*******************************************************************************
* 函 数 名         : MSG_Ctrl_Task
* 函数功能		     : 消息接收发送控制任务 周期性和触发性消息各可发送一个
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void MSG_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	s8 id_wait2send = -1;
	
	//遍历触发式消息发送任务
	for(i = 0; i < TG_MSG_NUM; i++)
	{
		//到了时间并且该轮还没有准备发送的msg帧
		if(-1 == id_wait2send && 1 == tg_msg_array[i].wait2send)
		{
			tg_msg_array[i].wait2send = 0;
			id_wait2send = i;
			break;
		}
	}
	TG_MSG_Frame_Send(id_wait2send);
	
	id_wait2send = -1;
	
	//遍历周期性消息发送任务
	for(i = 0; i < DT_MSG_NUM; i++)
	{
		//未到时间继续计时
		if(0 == dt_msg_array[i].wait2send)
		{
			dt_msg_array[i].time_cnt_ms += dT_ms;
			if(dt_msg_array[i].time_cnt_ms >= dt_msg_array[i].dt_ms)
			{
				dt_msg_array[i].time_cnt_ms = 0;
				dt_msg_array[i].wait2send = 1;
			}
		}
		//到了时间并且该轮还没有准备发送的msg帧
		else if(-1 == id_wait2send)
		{
			dt_msg_array[i].wait2send = 0;
			id_wait2send = i;
		}
	}
	DT_MSG_Frame_Send(id_wait2send);
}
	

/*******************************************************************************
* 函 数 名         : DT_MSG_Frame_Send
* 函数功能		     : 周期性消息帧发送
* 输    入         : 帧功能ID
* 输    出         : 无
*******************************************************************************/
void DT_MSG_Frame_Send(u8 fun_id)
{
	u8 buf[32];
	u8 cnt = 0;
	s16 s16_tmp;
	u16 u16_tmp;
	
	//构造帧头
	buf[cnt++] = FRAME_HEAD;
	buf[cnt++] = MCU_ADDR;
	buf[cnt++] = BOARDCAST_ADDR;
	buf[cnt++] = DT_MSG_ID_ARRAY[fun_id];
	
	switch(fun_id)
	{
		//包装mpu信息成消息帧并发送
		case MSG_ID_MPU:
		{
			buf[cnt++] = 13;
			buf[cnt++] = 0;
			
			s16_tmp = (s16)(imu_data.acc_x*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.acc_y*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.acc_z*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.gyro_x*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.gyro_y*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.gyro_z*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			buf[cnt++] = 1;
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装imu欧拉角信息成消息帧并发送
		case MSG_ID_IMU:
		{
			buf[cnt++] = 7;
			buf[cnt++] = 0;
			
			s16_tmp = (s16)(imu_data.rol*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.pit*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.yaw*100);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			buf[cnt++] = 0;
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装imu四元数信息成消息帧并发送
		case MSG_ID_IMU2:
		{
			buf[cnt++] = 9;
			buf[cnt++] = 0;
			
			s16_tmp = (s16)(imu_data.w*10000);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.x*10000);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.y*10000);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = (s16)(imu_data.z*10000);
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			
			buf[cnt++] = 0;
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装高度信息成消息帧并发送
		case MSG_ID_HIGHT:
		{

		}
		break;
		
		//包装速度信息成消息帧并发送
		case MSG_ID_SPD:
		{

		}
		break;
		
		//包装pwm信息成消息帧并发送
		case MSG_ID_PWM:
		{
			FLIGHT_Data_structure fl_data;
			//从消息队列中取出imu数据
			xQueuePeek(fl_data_queue, (void *)&fl_data, 0);	
			
			buf[cnt++] = 8;
			buf[cnt++] = 0;
			
			u16_tmp = (u16)(fl_data.pwm_out[CH_ROL] + 1500);
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			u16_tmp = (u16)(fl_data.pwm_out[CH_PIT] + 1500);
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			u16_tmp = (u16)(fl_data.pwm_out[CH_THR] + 1500);
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			u16_tmp = (u16)(fl_data.pwm_out[CH_YAW] + 1500);
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装gps信息成消息帧并发送
		case MSG_ID_GPS:
		{

		}
		break;
		
		//包装rc信息成消息帧并发送
		case MSG_ID_RC:
		{
			s16 ch_processed[16];
			//从消息队列中取出imu数据
			xQueuePeek(rc_data_queue, (void *)ch_processed, 0);	
			
			buf[cnt++] = 12;
			buf[cnt++] = 0;
			
			s16_tmp = ch_processed[CH_ROL] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = ch_processed[CH_PIT] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = ch_processed[CH_THR] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = ch_processed[CH_YAW] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = ch_processed[AUX1] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = ch_processed[AUX2] + 1500;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
	}
}

/*******************************************************************************
* 函 数 名         : TG_MSG_Frame_Send
* 函数功能		     : 触发性消息帧发送
* 输    入         : 帧功能ID
* 输    出         : 无
*******************************************************************************/
void TG_MSG_Frame_Send(u8 fun_id)
{
	u8 buf[64];
	u8 cnt = 0;
	s16 s16_tmp;
	u16 u16_tmp;
	float f_tmp;
	
	//构造帧头
	buf[cnt++] = FRAME_HEAD;
	buf[cnt++] = MCU_ADDR;
	buf[cnt++] = BOARDCAST_ADDR;
	buf[cnt++] = TG_MSG_ID_ARRAY[fun_id];
	
	switch(fun_id)
	{
		//包装参数数量信息成消息帧并发送
		case MSG_ID_PARNUM:
		{
			buf[cnt++] = 3;
			buf[cnt++] = 0;
			
			//CMD
			buf[cnt++] = 1;
			
			s16_tmp = PAR_NUM;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装参数值信息成消息帧并发送
		case MSG_ID_PAR:
		{
			buf[cnt++] = 6;
			buf[cnt++] = 0;
			
			u16_tmp = tg_msg_array[MSG_ID_PAR].msg_info;
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			f_tmp = (float)(fl_par.s16_unit_array[u16_tmp]);
			buf[cnt++] = BYTE0(f_tmp);
			buf[cnt++] = BYTE1(f_tmp);
			buf[cnt++] = BYTE2(f_tmp);
			buf[cnt++] = BYTE3(f_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装参数信息成消息帧并发送
		case MSG_ID_PARINFO:
		{
			buf[cnt++] = PAR_DESC_FRAME_LEN;
			buf[cnt++] = 0;
			
			u16_tmp = tg_msg_array[MSG_ID_PARINFO].msg_info;
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			f_tmp = par_info_array[u16_tmp].par_min;
			buf[cnt++] = BYTE0(f_tmp);
			buf[cnt++] = BYTE1(f_tmp);
			buf[cnt++] = BYTE2(f_tmp);
			buf[cnt++] = BYTE3(f_tmp);
			
			f_tmp = par_info_array[u16_tmp].par_max;
			buf[cnt++] = BYTE0(f_tmp);
			buf[cnt++] = BYTE1(f_tmp);
			buf[cnt++] = BYTE2(f_tmp);
			buf[cnt++] = BYTE3(f_tmp);
			
			for(u8 i = 0; i < PAR_NAME_LEN; i++)
			{
				buf[cnt++] = par_info_array[u16_tmp].name[i];
			}
			
			for(u8 i = 0; i < PAR_INFO_LEN; i++)
			{
				buf[cnt++] = par_info_array[u16_tmp].info[i];
			}
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装设备信息成消息帧并发送
		case MSG_ID_DEV:
		{
			buf[cnt++] = 9;
			buf[cnt++] = 0;
			
			buf[cnt++] = MCU_ADDR;
			
			s16_tmp = 0;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = 0;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = 0;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			s16_tmp = 0;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//包装参数写入应答信息成消息帧并发送
		case MSG_ID_ACK:
		{
			buf[cnt++] = 3;
			buf[cnt++] = 0;
			
			buf[cnt++] = 0xE1;
			
			u16_tmp = tg_msg_array[MSG_ID_ACK].msg_info;
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			//添加校验字段
			MSG_Check_ADD(buf, cnt);
			
			//发送
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
	}
}

/*******************************************************************************
* 函 数 名         : MSG_Check_ADD
* 函数功能		     : 往buf中添加最后的校验字段
* 输    入         : buf指针 需要计算校验的长度
* 输    出         : 无
*******************************************************************************/
void MSG_Check_ADD(u8 *buf, u8 len)
{
	u8 i = 0;
	u8 sum_check = 0;
	u8 add_check = 0;
	
	for(i = 0; i < len; i++)
	{
		sum_check += buf[i];
		add_check += sum_check;
	}
	
	buf[len++] = sum_check;
	buf[len++] = add_check;
}


/*******************************************************************************
* 函 数 名         : MSG_RECV_ByteGet
* 函数功能		     : MSG接收单字节解析
* 输    入         : 串口接收的一个字节数据
* 输    出         : 无
*******************************************************************************/
u8 msg_recv_buf[64];
void MSG_RECV_ByteGet(u8 data)
{
	static u8 state = 0;
	static u8 len = 0;
	
	if(state == 0)
	{
		if(data == 0xAB)
		{
			msg_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 3)
	{
		if(data == 0xE0 || data == 0xE1)
		{
			msg_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 4)
	{
		msg_recv_buf[state++] = data;
		len = data;
	}
	else if(state == len + 7)
	{
		msg_recv_buf[state++] = data;
		
		u8 sum_check = 0;
		u8 add_check = 0;
		for(u8 i = 0; i < len + 6; i++)
		{
			sum_check += msg_recv_buf[i];
			add_check += sum_check;
		}
		
		if(sum_check == msg_recv_buf[len+6] && add_check == msg_recv_buf[len+7])
		{
			MSG_RECV_Analysis(len);
		}
		
		state = 0;
	}
	else
	{
		msg_recv_buf[state++] = data;
	}	
}

/*******************************************************************************
* 函 数 名         : MSG_RECV_Analysis
* 函数功能		     : 消息帧解析函数
* 输    入         : 有效数据长度
* 输    出         : 无
*******************************************************************************/
void MSG_RECV_Analysis(u8 len)
{
	if(msg_recv_buf[3] == 0xE0)
	{
		//获取设备信息
		if(msg_recv_buf[6] == 0)
		{
			tg_msg_array[MSG_ID_DEV].wait2send = 1;
		}
		//获取参数数量
		else if(msg_recv_buf[6] == 1)
		{
			tg_msg_array[MSG_ID_PARNUM].wait2send = 1;
		}
		//获取参数值 msg_info为目标参数的ID
		else if(msg_recv_buf[6] == 2)
		{
			tg_msg_array[MSG_ID_PAR].wait2send = 1;
			tg_msg_array[MSG_ID_PAR].msg_info = *((u16 *)&(msg_recv_buf[7]));
		}
		//获取参数信息 msg_info为目标参数的ID
		else if(msg_recv_buf[6] == 3)
		{
			tg_msg_array[MSG_ID_PARINFO].wait2send = 1;
			tg_msg_array[MSG_ID_PARINFO].msg_info = *((u16 *)&(msg_recv_buf[7]));
		}
	}
	//修改参数
	else if(msg_recv_buf[3] == 0xE1)
	{
		u16 per_id;
		float per_val;
		
		//msg_info为目标参数的ID
		tg_msg_array[MSG_ID_ACK].wait2send = 1;
		tg_msg_array[MSG_ID_ACK].msg_info = *((u16 *)&(msg_recv_buf[12]));
		per_id = *((u16 *)&(msg_recv_buf[6]));
		per_val = *((float *)&(msg_recv_buf[8]));
		PAR_Change(per_id, per_val);
	}
}

///*******************************************************************************
//* 函 数 名         : MSG_Send
//* 函数功能		     : 消息发送到上位机
//* 输    入         : 无
//* 输    出         : 1 失败 0 成功
//*******************************************************************************/
//void MSG_Send(void)
//{
//	IMU_Data_structure imu_data;
//	u8 string[60] = {0,0,0,0,
//					 0,0,0,0,
//                     0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//					 0,0,0,0,
//		             0x00,0x00,0x80,0x7f};
//	float *realData = (float *)(string);
//	
//	//从消息队列中取出imu数据
//	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);					 
//	
//	*(realData + 0) = imu_data.w;
//	*(realData + 1) = imu_data.x;
//	*(realData + 2) = imu_data.y;
//	*(realData + 3) = imu_data.z;
//					 
//	*(realData + 4) = rotation_data.w;
//	*(realData + 5) = rotation_data.x;
//	*(realData + 6) = rotation_data.y;
//	*(realData + 7) = rotation_data.z;
//					 
//	*(realData + 8) = rotation_data.w_err;
//	*(realData + 9) = rotation_data.x_err;	
//	*(realData + 10) = rotation_data.y_err;
//	*(realData + 11) = rotation_data.z_err;
//					 
//	*(realData + 12) = rotation_data.expect_pit_err;
//	*(realData + 13) = rotation_data.expect_rol_err;
//					
//	DRV_USART1_Send(string, 60);
//}
