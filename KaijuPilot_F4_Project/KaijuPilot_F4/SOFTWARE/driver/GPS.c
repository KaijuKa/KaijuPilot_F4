#include "GPS.h"
#include "pos_calcu.h"

u8 gps_recv_buf[128];
/*******************************************************************************
* 函 数 名         : DRV_GPS_ByteGet
* 函数功能		     : GPS接收单字节解析
* 输    入         : 串口接收的一个字节数据
* 输    出         : 无
*******************************************************************************/
void DRV_GPS_ByteGet(u8 data)
{
	static u8 state = 0;
	static u8 len = 0;
	
	if(state == 0)
	{
		if(data == 0xB5)
		{
			gps_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 1)
	{
		if(data == 0x62)
		{
			gps_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 2)
	{
		if(data == 0x01)
		{
			gps_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 3)
	{
		if(data == 0x07)
		{
			gps_recv_buf[state++] = data;
		}
		else
		{
			state = 0;
		}
	}
	else if(state == 4)
	{
		gps_recv_buf[state++] = data;
		len = data;
		
		if(len > 100)
		{
			state = 0;
		}
	}
	else if(state == len + 7)
	{
		u8 a_check = 0;
		for(u8 i = 2; i < len + 6; i++)
		{
			a_check += gps_recv_buf[i];
		}
		
		if(a_check == gps_recv_buf[len + 6])
		{
			DRV_GPS_Analysis(len);
		}
		
		state = 0;
	}
	else
	{
		gps_recv_buf[state++] = data;
	}
}

u32 time_cnt;
/*******************************************************************************
* 函 数 名         : DRV_GPS_Analysis
* 函数功能		     : GPS解析函数
* 输    入         : 有效数据长度
* 输    出         : 无
*******************************************************************************/
void DRV_GPS_Analysis(u8 len)
{
	u8 tmp[4];

	time_cnt ++;
	
	pos_data.fix_sta = gps_recv_buf[27];
	
	pos_data.star_num = gps_recv_buf[29];
	
	tmp[0] = gps_recv_buf[30];
	tmp[1] = gps_recv_buf[31];
	tmp[2] = gps_recv_buf[32];
	tmp[3] = gps_recv_buf[33];
	pos_data.log = (*((s32 *)(tmp)))/10000000.0f;
	
	tmp[0] = gps_recv_buf[34];
	tmp[1] = gps_recv_buf[35];
	tmp[2] = gps_recv_buf[36];
	tmp[3] = gps_recv_buf[37];
	pos_data.lat = (*((s32 *)(tmp)))/10000000.0f;
	
	tmp[0] = gps_recv_buf[42];
	tmp[1] = gps_recv_buf[43];
	tmp[2] = gps_recv_buf[44];
	tmp[3] = gps_recv_buf[45];
	pos_data.hMSL = (*((s32 *)(tmp)));
	pos_data.hMSL /= 10.0f; //cm
	
	tmp[0] = gps_recv_buf[46];
	tmp[1] = gps_recv_buf[47];
	tmp[2] = gps_recv_buf[48];
	tmp[3] = gps_recv_buf[49];
	pos_data.hAcc = (*((u32 *)(tmp)));
	pos_data.hAcc /= 10.0f; //cm
	
	tmp[0] = gps_recv_buf[50];
	tmp[1] = gps_recv_buf[51];
	tmp[2] = gps_recv_buf[52];
	tmp[3] = gps_recv_buf[53];
	pos_data.cAcc = (*((u32 *)(tmp)));
	pos_data.cAcc /= 10.0f; //cm
	
	tmp[0] = gps_recv_buf[54];
	tmp[1] = gps_recv_buf[55];
	tmp[2] = gps_recv_buf[56];
	tmp[3] = gps_recv_buf[57];
	pos_data.velN = (*((s32 *)(tmp)));
	pos_data.velN /= 10.0f; //cm/s
	
	tmp[0] = gps_recv_buf[58];
	tmp[1] = gps_recv_buf[59];
	tmp[2] = gps_recv_buf[60];
	tmp[3] = gps_recv_buf[61];
	pos_data.velE = (*((s32 *)(tmp)));
	pos_data.velE /= 10.0f; //cm/s
	
	tmp[0] = gps_recv_buf[66];
	tmp[1] = gps_recv_buf[67];
	tmp[2] = gps_recv_buf[68];
	tmp[3] = gps_recv_buf[69];
	pos_data.gSpeed = (*((s32 *)(tmp)));
	pos_data.gSpeed /= 10.0f; //cm/s
	
	tmp[0] = gps_recv_buf[70];
	tmp[1] = gps_recv_buf[71];
	tmp[2] = gps_recv_buf[72];
	tmp[3] = gps_recv_buf[73];
	pos_data.headMot = (*((s32 *)(tmp))) / 100000.0f;
	
	tmp[0] = gps_recv_buf[74];
	tmp[1] = gps_recv_buf[75];
	tmp[2] = gps_recv_buf[76];
	tmp[3] = gps_recv_buf[77];
	pos_data.spdAcc = (*((u32 *)(tmp)));
	pos_data.spdAcc /= 10.0f; //cm/s
	
	tmp[0] = gps_recv_buf[78];
	tmp[1] = gps_recv_buf[79];
	tmp[2] = gps_recv_buf[80];
	tmp[3] = gps_recv_buf[81];
	pos_data.headAcc = (*((u32 *)(tmp))) / 100000.0f;
	
	tmp[0] = gps_recv_buf[82];
	tmp[1] = gps_recv_buf[83];
	tmp[2] = 0x00;
	tmp[3] = 0x00;
	pos_data.pDop = (*((u16 *)(tmp))) / 100.0f;
}
