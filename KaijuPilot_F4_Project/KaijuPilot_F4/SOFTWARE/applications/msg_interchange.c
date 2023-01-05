#include "msg_interchange.h"
#include "imu.h"
#include "uart.h"
#include "remote_signal.h"
#include "att_ctrl.h"
#include "flight_ctrl.h"
#include "par_manage.h"

#include "FreeRTOS.h"
#include "queue.h"

//��������Ϣ֡����
MSG_Ctrl_structure dt_msg_array[DT_MSG_NUM];
//��������Ϣ֡����
MSG_Ctrl_structure tg_msg_array[TG_MSG_NUM];

//ö�ٶ�Ӧ����ʵ��֡ID
u8 DT_MSG_ID_ARRAY[DT_MSG_NUM] = {0x01, 0x03, 0x04, 0x05, 0x07, 0x20, 0x30, 0x40};

u8 TG_MSG_ID_ARRAY[TG_MSG_NUM] = {0xE0, 0xE1, 0xE2, 0xE3, 0x00};

/*******************************************************************************
* �� �� ��         : DT_MSG_Init
* ��������		     : ��������Ϣ���Ϳ��ƽṹ���ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DT_MSG_Init(void)
{
	//���Դ���������
	dt_msg_array[MSG_ID_MPU].dt_ms = 20;
	
	//ŷ����
	dt_msg_array[MSG_ID_IMU].dt_ms = 20;
	
	//��Ԫ��
	dt_msg_array[MSG_ID_IMU2].dt_ms = 20;
	
	//�߶�
	dt_msg_array[MSG_ID_HIGHT].dt_ms = 200;
	
	//�����ٶ�
	dt_msg_array[MSG_ID_SPD].dt_ms = 200;
	
	//pwm���
	dt_msg_array[MSG_ID_PWM].dt_ms = 20;
	
	//GPSλ��
	dt_msg_array[MSG_ID_GPS].dt_ms = 200;
	
	//ң����
	dt_msg_array[MSG_ID_RC].dt_ms = 20;
}

/*******************************************************************************
* �� �� ��         : MSG_Ctrl_Task
* ��������		     : ��Ϣ���շ��Ϳ������� �����Ժʹ�������Ϣ���ɷ���һ��
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void MSG_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	s8 id_wait2send = -1;
	
	//��������ʽ��Ϣ��������
	for(i = 0; i < TG_MSG_NUM; i++)
	{
		//����ʱ�䲢�Ҹ��ֻ�û��׼�����͵�msg֡
		if(-1 == id_wait2send && 1 == tg_msg_array[i].wait2send)
		{
			tg_msg_array[i].wait2send = 0;
			id_wait2send = i;
			break;
		}
	}
	TG_MSG_Frame_Send(id_wait2send);
	
	id_wait2send = -1;
	
	//������������Ϣ��������
	for(i = 0; i < DT_MSG_NUM; i++)
	{
		//δ��ʱ�������ʱ
		if(0 == dt_msg_array[i].wait2send)
		{
			dt_msg_array[i].time_cnt_ms += dT_ms;
			if(dt_msg_array[i].time_cnt_ms >= dt_msg_array[i].dt_ms)
			{
				dt_msg_array[i].time_cnt_ms = 0;
				dt_msg_array[i].wait2send = 1;
			}
		}
		//����ʱ�䲢�Ҹ��ֻ�û��׼�����͵�msg֡
		else if(-1 == id_wait2send)
		{
			dt_msg_array[i].wait2send = 0;
			id_wait2send = i;
		}
	}
	DT_MSG_Frame_Send(id_wait2send);
}
	

/*******************************************************************************
* �� �� ��         : DT_MSG_Frame_Send
* ��������		     : ��������Ϣ֡����
* ��    ��         : ֡����ID
* ��    ��         : ��
*******************************************************************************/
void DT_MSG_Frame_Send(u8 fun_id)
{
	u8 buf[32];
	u8 cnt = 0;
	s16 s16_tmp;
	u16 u16_tmp;
	
	//����֡ͷ
	buf[cnt++] = FRAME_HEAD;
	buf[cnt++] = MCU_ADDR;
	buf[cnt++] = BOARDCAST_ADDR;
	buf[cnt++] = DT_MSG_ID_ARRAY[fun_id];
	
	switch(fun_id)
	{
		//��װmpu��Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װimuŷ������Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װimu��Ԫ����Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װ�߶���Ϣ����Ϣ֡������
		case MSG_ID_HIGHT:
		{

		}
		break;
		
		//��װ�ٶ���Ϣ����Ϣ֡������
		case MSG_ID_SPD:
		{

		}
		break;
		
		//��װpwm��Ϣ����Ϣ֡������
		case MSG_ID_PWM:
		{
			FLIGHT_Data_structure fl_data;
			//����Ϣ������ȡ��imu����
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װgps��Ϣ����Ϣ֡������
		case MSG_ID_GPS:
		{

		}
		break;
		
		//��װrc��Ϣ����Ϣ֡������
		case MSG_ID_RC:
		{
			s16 ch_processed[16];
			//����Ϣ������ȡ��imu����
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
	}
}

/*******************************************************************************
* �� �� ��         : TG_MSG_Frame_Send
* ��������		     : ��������Ϣ֡����
* ��    ��         : ֡����ID
* ��    ��         : ��
*******************************************************************************/
void TG_MSG_Frame_Send(u8 fun_id)
{
	u8 buf[64];
	u8 cnt = 0;
	s16 s16_tmp;
	u16 u16_tmp;
	float f_tmp;
	
	//����֡ͷ
	buf[cnt++] = FRAME_HEAD;
	buf[cnt++] = MCU_ADDR;
	buf[cnt++] = BOARDCAST_ADDR;
	buf[cnt++] = TG_MSG_ID_ARRAY[fun_id];
	
	switch(fun_id)
	{
		//��װ����������Ϣ����Ϣ֡������
		case MSG_ID_PARNUM:
		{
			buf[cnt++] = 3;
			buf[cnt++] = 0;
			
			//CMD
			buf[cnt++] = 1;
			
			s16_tmp = PAR_NUM;
			buf[cnt++] = BYTE0(s16_tmp);
			buf[cnt++] = BYTE1(s16_tmp);
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װ����ֵ��Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װ������Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װ�豸��Ϣ����Ϣ֡������
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
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
		
		//��װ����д��Ӧ����Ϣ����Ϣ֡������
		case MSG_ID_ACK:
		{
			buf[cnt++] = 3;
			buf[cnt++] = 0;
			
			buf[cnt++] = 0xE1;
			
			u16_tmp = tg_msg_array[MSG_ID_ACK].msg_info;
			buf[cnt++] = BYTE0(u16_tmp);
			buf[cnt++] = BYTE1(u16_tmp);
			
			//���У���ֶ�
			MSG_Check_ADD(buf, cnt);
			
			//����
			DRV_USART1_Send(buf, cnt+2);
		}
		break;
	}
}

/*******************************************************************************
* �� �� ��         : MSG_Check_ADD
* ��������		     : ��buf���������У���ֶ�
* ��    ��         : bufָ�� ��Ҫ����У��ĳ���
* ��    ��         : ��
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
* �� �� ��         : MSG_RECV_ByteGet
* ��������		     : MSG���յ��ֽڽ���
* ��    ��         : ���ڽ��յ�һ���ֽ�����
* ��    ��         : ��
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
* �� �� ��         : MSG_RECV_Analysis
* ��������		     : ��Ϣ֡��������
* ��    ��         : ��Ч���ݳ���
* ��    ��         : ��
*******************************************************************************/
void MSG_RECV_Analysis(u8 len)
{
	if(msg_recv_buf[3] == 0xE0)
	{
		//��ȡ�豸��Ϣ
		if(msg_recv_buf[6] == 0)
		{
			tg_msg_array[MSG_ID_DEV].wait2send = 1;
		}
		//��ȡ��������
		else if(msg_recv_buf[6] == 1)
		{
			tg_msg_array[MSG_ID_PARNUM].wait2send = 1;
		}
		//��ȡ����ֵ msg_infoΪĿ�������ID
		else if(msg_recv_buf[6] == 2)
		{
			tg_msg_array[MSG_ID_PAR].wait2send = 1;
			tg_msg_array[MSG_ID_PAR].msg_info = *((u16 *)&(msg_recv_buf[7]));
		}
		//��ȡ������Ϣ msg_infoΪĿ�������ID
		else if(msg_recv_buf[6] == 3)
		{
			tg_msg_array[MSG_ID_PARINFO].wait2send = 1;
			tg_msg_array[MSG_ID_PARINFO].msg_info = *((u16 *)&(msg_recv_buf[7]));
		}
	}
	//�޸Ĳ���
	else if(msg_recv_buf[3] == 0xE1)
	{
		u16 per_id;
		float per_val;
		
		//msg_infoΪĿ�������ID
		tg_msg_array[MSG_ID_ACK].wait2send = 1;
		tg_msg_array[MSG_ID_ACK].msg_info = *((u16 *)&(msg_recv_buf[12]));
		per_id = *((u16 *)&(msg_recv_buf[6]));
		per_val = *((float *)&(msg_recv_buf[8]));
		PAR_Change(per_id, per_val);
	}
}

///*******************************************************************************
//* �� �� ��         : MSG_Send
//* ��������		     : ��Ϣ���͵���λ��
//* ��    ��         : ��
//* ��    ��         : 1 ʧ�� 0 �ɹ�
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
//	//����Ϣ������ȡ��imu����
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
