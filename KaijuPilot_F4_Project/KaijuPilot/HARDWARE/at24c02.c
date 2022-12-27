#include "at24c02.h"
#include "iic.h"

#define WRITE_COMMAND 0xA0                       /* ����24C02��������ַSLA�ͷ���λW */
#define READ_COMMAND  0xA1                       /* ����24C02��������ַSLA�ͷ���λR */

/*******************************************************************************
* �� �� ��         : AT24_Read_Str
* ��������		     : ��AT24�ж���һ���ַ���
* ��    ��         : str �ַ������� ��ʼ��ַ
* ��    ��         : ��
*******************************************************************************/
void AT24_Read_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	//���Ϳ�ʼ�źż�д����
	IIC_Start();
	IIC_Send_Byte(WRITE_COMMAND);
	
	//���Ͷ�д������ʼ�洢��ַ
	IIC_Send_Byte(addr);
	
	//���¿�ʼ���Ͷ�����
	IIC_Start();
	IIC_Send_Byte(READ_COMMAND);
	
	//������ȡlen���ֽ�
	for(i = 0; i < len; i++)
	{
		str[i] = IIC_Read_Byte(1);
	}
	//����
	IIC_Stop();
}

/*******************************************************************************
* �� �� ��         : AT24_Write_Str
* ��������		     : ��AT24д��һ���ַ���
* ��    ��         : str �ַ������� ��ʼ��ַ
* ��    ��         : ��
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