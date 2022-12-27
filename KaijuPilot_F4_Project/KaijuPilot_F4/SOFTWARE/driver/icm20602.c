#include "icm20602.h"
#include "spi.h"
#include "delay.h"

/*******************************************************************************
* 函 数 名         : icm20602_enable
* 函数功能		   : icm20602片选
* 输    入         : 1 片选 0 不片选
* 输    出         : 无
*******************************************************************************/
static void icm20602_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
	else
		GPIO_SetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
}

/*******************************************************************************
* 函 数 名         : icm20602_readbuf
* 函数功能		   : 在icm20602中从指定寄存器开始读入length长度的数据
* 输    入         : reg起始寄存器地址 length数据长度 data数据存储地址
* 输    出         : 无
*******************************************************************************/
static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	icm20602_enable(1);
	DRV_SPI_Read_Write_Byte(reg|0x80);
	DRV_SPI_Receive(data,length);
	icm20602_enable(0);
}

/*******************************************************************************
* 函 数 名         : icm20602_writebyte
* 函数功能		   : 在icm20602中从reg开始写入一个字节数据
* 输    入         : reg起始寄存器地址 data字节数据
* 输    出         : 读出的一个字节数据
*******************************************************************************/
static u8 icm20602_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	icm20602_enable(1);
	status = DRV_SPI_Read_Write_Byte(reg);
	DRV_SPI_Read_Write_Byte(data);
	icm20602_enable(0);
	return status;
}

/*******************************************************************************
* 函 数 名         : DRV_Icm20602_Init
* 函数功能		   : 配置icm20602
* 输    入         : 无
* 输    出         : 0成功 1失败
*******************************************************************************/
u8 ICM_ID;
u8 DRV_Icm20602_Init(void)
{
	u8 tmp;
	
	//复位内部寄存器和装载默认值
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	delay_ms(10);
	//设置icm20602时钟为自适应
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	delay_ms(10);
	
	//获取设备id
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	return 0;

	//复位acc temp 数字信号通路
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	delay_ms(10);
    //复位gyro acc temp 数字信号通路
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	delay_ms(10);
		
	//关闭iic
	icm20602_writebyte(MPU_RA_DMP_CFG_1,0x40);
	delay_ms(10);
	//开启3轴acc 3轴gyro
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	delay_ms(10);
	//分频系数为0 则数据输出频率为1KHz
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	delay_ms(10);

	//设置为20Hz lpf
	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	delay_ms(10);
	//设置gyro精度为+-2000dps
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	delay_ms(10);
	//设置acc精度为+-8g
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
	delay_ms(10);
	//加速度计LPF 20HZ
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG2,0x04);
	delay_ms(10);
	//关闭低功耗
	icm20602_writebyte(MPU_RA_LP_MODE_CFG,0x00);
	delay_ms(10);
	//关闭FIFO
	icm20602_writebyte(MPU_RA_FIFO_EN,0x00);
	delay_ms(10);
	//关闭中断响应
	icm20602_writebyte(MPUREG_INT_PIN_CFG,0x00);
	delay_ms(10);
	//关闭中断
	icm20602_writebyte(MPUREG_INT_ENABLE,0x00);
	delay_ms(10);

	//读取ID
	icm20602_readbuf(MPU_RA_WHO_AM_I, 1, &ICM_ID);
	//
	if(ICM_ID == 0X12)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/*******************************************************************************
* 函 数 名         : DRV_Icm20602_Read
* 函数功能		   : 读取mpu数据
* 输    入         : 数据存储buf
* 输    出         : 无
*******************************************************************************/
void DRV_Icm20602_Read(u8 *mpu_buffer)
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}
