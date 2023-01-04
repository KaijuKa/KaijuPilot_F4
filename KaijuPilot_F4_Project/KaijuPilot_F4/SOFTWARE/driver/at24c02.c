#include "at24c02.h"
#include "iic.h"

#define WRITE_COMMAND 0xA0                       /* ����24C02��������ַSLA�ͷ���λW */
#define READ_COMMAND  0xA1                       /* ����24C02��������ַSLA�ͷ���λR */

/*******************************************************************************
* �� �� ��         : DRV_AT24_Read_Str
* ��������		     : ��AT24�ж���һ���ַ���
* ��    ��         : str �ַ������� ��ʼ��ַ
* ��    ��         : ��
*******************************************************************************/
void DRV_AT24_Read_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	//���Ϳ�ʼ�źż�д����
	DRV_IIC_Start();
	DRV_IIC_Send_Byte(WRITE_COMMAND);
	
	//���Ͷ�д������ʼ�洢��ַ
	DRV_IIC_Send_Byte(addr);
	
	//���¿�ʼ���Ͷ�����
	DRV_IIC_Start();
	DRV_IIC_Send_Byte(READ_COMMAND);
	
	//������ȡlen���ֽ�
	for(i = 0; i < len; i++)
	{
		str[i] = DRV_IIC_Read_Byte(1);
	}
	//����
	DRV_IIC_Stop();
}

/*******************************************************************************
* �� �� ��         : DRV_AT24_Write_Str
* ��������		     : ��AT24д��һ���ַ���
* ��    ��         : str �ַ������� ��ʼ��ַ
* ��    ��         : ��
*******************************************************************************/
void DRV_AT24_Write_Str(u8* str, u8 len, u8 addr)
{
	u8 i;
	
	for(i = 0; i < len; i++)
	{
		DRV_AT24_Write_Byte(str[i], addr+i);
		delay_ms(5);
	}
}

/*******************************************************************************
* �� �� ��         : DRV_AT24_Write_Byte
* ��������		     : ��AT24д��һ���ֽ�
* ��    ��         : byte ��ַ
* ��    ��         : ��
*******************************************************************************/
void DRV_AT24_Write_Byte(u8 byte, u8 addr)
{
	DRV_IIC_Start();
	DRV_IIC_Send_Byte(WRITE_COMMAND);
	DRV_IIC_Send_Byte(addr);
	DRV_IIC_Send_Byte(byte);
	DRV_IIC_Stop();
}
