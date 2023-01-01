#include "at24c02.h"
#include "iic.h"

#define WRITE_COMMAND 0xA0                       /* 定义24C02的器件地址SLA和方向位W */
#define READ_COMMAND  0xA1                       /* 定义24C02的器件地址SLA和方向位R */

/*******************************************************************************
* 函 数 名         : DRV_AT24_Read_Str
* 函数功能		     : 从AT24中读出一串字符串
* 输    入         : str 字符串长度 起始地址
* 输    出         : 无
*******************************************************************************/
void DRV_AT24_Read_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	//发送开始信号及写命令
	DRV_IIC_Start();
	DRV_IIC_Send_Byte(WRITE_COMMAND);
	
	//发送读写操作起始存储地址
	DRV_IIC_Send_Byte(addr);
	
	//重新开始发送读命令
	DRV_IIC_Start();
	DRV_IIC_Send_Byte(READ_COMMAND);
	
	//持续读取len个字节
	for(i = 0; i < len; i++)
	{
		str[i] = DRV_IIC_Read_Byte(1);
	}
	//结束
	DRV_IIC_Stop();
}

/*******************************************************************************
* 函 数 名         : DRV_AT24_Write_Str
* 函数功能		     : 向AT24写入一串字符串
* 输    入         : str 字符串长度 起始地址
* 输    出         : 无
*******************************************************************************/
void DRV_AT24_Write_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	for(i = 0; i < len; i++)
	{
		DRV_IIC_Start();
		DRV_IIC_Send_Byte(WRITE_COMMAND);
		DRV_IIC_Send_Byte(addr+i);
		DRV_IIC_Send_Byte(str[i]);
		DRV_IIC_Stop();
		delay_ms(5);
	}
}
