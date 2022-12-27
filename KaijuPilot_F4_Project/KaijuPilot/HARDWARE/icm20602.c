#include "icm20602.h"
#include "spi.h"
#include "delay.h"

/**
  * @brief :icm20602 csƬѡ����
  * @param :
  *         @ena:�Ƿ�Ƭѡ 1Ƭѡ 0��Ƭѡ
  * @note  :��
  * @retval:��
  */
static void icm20602_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
	else
		GPIO_SetBits(ICM_CS_GPIO_PORT, ICM_CS_GPIO_PIN);
}

/**
  * @brief :��icm20602��ȡ����length������
  * @param :
  *			@reg:��ȡ�ֽڻ�������ַ
  *			@length:�ֽڳ���
  *			@data:��ȡ�Ļ�����
  * @note  :��
  * @retval:��
  */
static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
	icm20602_enable(1);
	drv_spi_read_write_byte(reg|0x80);
	drv_spi_receive(data,length);
	icm20602_enable(0);
}

/**
  * @brief :��icm20602д��һ���ֽڵ�����
  * @param :
  *			@reg:д���ֽڻ�������ַ
  *			@data:д����ֽ�����
  * @note  :��
  * @retval:��
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
  * @brief :��ʼ��icm20602
  * @param :��
  * @note  :��
  * @retval:��
  */
u8 ICM_ID;
u8 Drv_Icm20602_Init(void)
{
	u8 tmp;
	
	//��λ�ڲ��Ĵ�����װ��Ĭ��ֵ
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
	delay_ms(10);
	//����icm20602ʱ��Ϊ����Ӧ
	icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
	delay_ms(10);
	
	//��ȡ�豸id
	icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);
	if(tmp != MPU_WHOAMI_20602)
	return 0;

	//��λacc temp �����ź�ͨ·
	icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
	delay_ms(10);
    //��λgyro acc temp �����ź�ͨ·
	icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
	delay_ms(10);
		
	//�ر�iic
	icm20602_writebyte(MPU_RA_DMP_CFG_1,0x40);
	delay_ms(10);
	//����3��acc 3��gyro
	icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
	delay_ms(10);
	//��Ƶϵ��Ϊ0 ���������Ƶ��Ϊ1KHz
	icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);
	delay_ms(10);

	//����Ϊ20Hz lpf
	icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
	delay_ms(10);
	//����gyro����Ϊ+-2000dps
	icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
	delay_ms(10);
	//����acc����Ϊ+-8g
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
	delay_ms(10);
	//���ٶȼ�LPF 20HZ
	icm20602_writebyte(MPU_RA_ACCEL_CONFIG2,0x04);
	delay_ms(10);
	//�رյ͹���
	icm20602_writebyte(MPU_RA_LP_MODE_CFG,0x00);
	delay_ms(10);
	//�ر�FIFO
	icm20602_writebyte(MPU_RA_FIFO_EN,0x00);
	delay_ms(10);
	//�ر��ж���Ӧ
	icm20602_writebyte(MPUREG_INT_PIN_CFG,0x00);
	delay_ms(10);
	//�ر��ж�
	icm20602_writebyte(MPUREG_INT_ENABLE,0x00);
	delay_ms(10);

	//��ȡID
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
  * @brief :��ȡ����������
  * @param :
			mpu_buffer:����buffer
  * @note  :��
  * @retval:��
  */
void Drv_Icm20602_Read(u8 *mpu_buffer)
{
	icm20602_readbuf(MPUREG_ACCEL_XOUT_H,14,mpu_buffer);
}
