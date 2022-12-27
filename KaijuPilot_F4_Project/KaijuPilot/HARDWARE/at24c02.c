#include "at24c02.h"
#include "iic.h"

#define WRITE_COMMAND 0xA0                       /* 定义24C02的器件地址SLA和方向位W */
#define READ_COMMAND  0xA1                       /* 定义24C02的器件地址SLA和方向位R */

/*******************************************************************************
* 函 数 名         : AT24_Read_Str
* 函数功能		     : 从AT24中读出一串字符串
* 输    入         : str 字符串长度 起始地址
* 输    出         : 无
*******************************************************************************/
void AT24_Read_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	//发送开始信号及写命令
	IIC_Start();
	IIC_Send_Byte(WRITE_COMMAND);
	
	//发送读写操作起始存储地址
	IIC_Send_Byte(addr);
	
	//重新开始发送读命令
	IIC_Start();
	IIC_Send_Byte(READ_COMMAND);
	
	//持续读取len个字节
	for(i = 0; i < len; i++)
	{
		str[i] = IIC_Read_Byte(1);
	}
	//结束
	IIC_Stop();
}

/*******************************************************************************
* 函 数 名         : AT24_Write_Str
* 函数功能		     : 向AT24写入一串字符串
* 输    入         : str 字符串长度 起始地址
* 输    出         : 无
*******************************************************************************/
void AT24_Write_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	for(i = 0; i < len; i++)
	{
		IIC_Start();
		IIC_Send_Byte(WRITE_COMMAND);
		IIC_Send_Byte(addr);
		IIC_Send_Byte(str[i]);
		IIC_Stop();
	}
}