#include "msg_interchange.h"
#include "imu.h"
#include "uart.h"
#include "remote_signal.h"

#include "FreeRTOS.h"
#include "queue.h"

/*******************************************************************************
* 函 数 名         : MSG_Send
* 函数功能		     : 消息发送到上位机
* 输    入         : 无
* 输    出         : 1 失败 0 成功
*******************************************************************************/
void MSG_Send(void)
{
	IMU_Data_structure imu_data;
	u8 string[48] = {0,0,0,0,
					 0,0,0,0,
                     0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
					 0,0,0,0,
		             0x00,0x00,0x80,0x7f};
	float *realData = (float *)(string);
	
	//从消息队列中取出imu数据
	xQueuePeek(imu_data_queue, (void *)&imu_data, 0);					 
	
	*(realData + 0) = imu_data.pit;
	*(realData + 1) = imu_data.rol;
	*(realData + 2) = imu_data.yaw;
	
	*(realData + 3) = imu_data.gyro_x;
	*(realData + 4) = imu_data.gyro_y;
	*(realData + 5) = imu_data.gyro_z;
					 
	*(realData + 6) = rc_data.ch_processed[CH_ROL];
	*(realData + 7) = rc_data.ch_processed[CH_PIT];
	*(realData + 8) = rc_data.ch_processed[CH_THR];	
	*(realData + 9) = rc_data.ch_processed[CH_YAW];
	*(realData + 10) = rc_data.ch_processed[AUX1];
	DRV_USART1_Send(string, 48);
}
