#include "icm20602.h"
#include "spi.h"
#include "delay.h"

/**
  * @brief :icm20602 cs片选控制
  * @param :
  *         @ena:是否片选 1片选 0不片选
  * @note  :无
  * @retval:无
  */
static void icm20602_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
	else
		GPIO_SetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
}

/**
  * @brief :从icm20602读取连续length的数据
  * @param :
  *			@reg:读取字节缓冲区地址
  *			@length:字节长度
  *			@data:读取的缓冲区
  * @note  :无
  * @retval:无
  */
static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	icm20602_enable(1);
	drv_spi_read_write_byte(reg|0x80);
	drv_spi_receive(data,length);
	icm20602_enable(0);
}

/**
  * @brief :向icm20602写入一个字节的数据
  * @param :
  *			@reg:写入字节缓冲区地址
  *			@data:写入的字节数据
  * @note  :无
  * @retval:无
  */
static u8 icm20602_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	icm20602_enable(1);
	status = drv_spi_read_write_byte(reg);
	drv_spi_read_write_byte(data);
	icm20602_enable(0);
	return status;
}

/**
  * @brief :初始化icm20602
  * @param :无
  * @note  :无
  * @retval:无
  */
u8 ICM_ID;
u8 Drv_Icm20602_Init(void)
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
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief :获取传感器数据
  * @param :
			mpu_buffer:数据buffer
  * @note  :无
  * @retval:无
  */
void Drv_Icm20602_Read(u8 *mpu_buffer)
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}
