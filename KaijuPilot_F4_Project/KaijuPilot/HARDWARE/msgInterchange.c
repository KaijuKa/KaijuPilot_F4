#include "msgInterchange.h"
#include "imu.h"
#include "uart.h"
#include "remote_signal.h"

/**
  * @brief :imu解析数据发送
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void imu_send(void)
{
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
	*(realData + 0) = imu_data.pit;
	*(realData + 1) = imu_data.rol;
	*(realData + 2) = imu_data.yaw;
	
	*(realData + 3) = imu_data.gyro_x;
	*(realData + 4) = imu_data.gyro_y;
	*(realData + 5) = imu_data.gyro_z;
					 
	*(realData + 6) = rc_in.ch_processed[CH_ROL];
	*(realData + 7) = rc_in.ch_processed[CH_PIT];
	*(realData + 8) = rc_in.ch_processed[CH_THR];	
	*(realData + 9) = rc_in.ch_processed[CH_YAW];
	*(realData + 10) = rc_in.ch_processed[AUX1];
	Uart1_Send(string, 48);
}
