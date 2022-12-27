#include "spi.h"

/** Ӳ��SPI */
#define SPI_WAIT_TIMEOUT			((uint16_t)0xFFFF)

/**
  * @brief :SPI��ʼ��(Ӳ��)
  * @param :��
  * @note  :��
  * @retval:��
  */ 
void drv_spi_init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI�������� */
	RCC_APB2PeriphClockCmd( SPI_CLK_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_MOSI_GPIO_CLK | SPI_NSS_GPIO_CLK, ENABLE );	//�򿪶˿�ʱ��
	
	//SCK MOSI MISO ����Ϊ����
	SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_50MHz;
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF_PP;
	
	SpiGpioInitStructer.GPIO_Pin = SPI_CLK_GPIO_PIN;
	GPIO_Init( SPI_CLK_GPIO_PORT, &SpiGpioInitStructer );		//��ʼ��SCK
	
	SpiGpioInitStructer.GPIO_Pin = SPI_MOSI_GPIO_PIN;
	GPIO_Init( SPI_MOSI_GPIO_PORT, &SpiGpioInitStructer );		//��ʼ��MOSI
	
	SpiGpioInitStructer.GPIO_Pin = SPI_MISO_GPIO_PIN;
	GPIO_Init( SPI_MISO_GPIO_PORT, &SpiGpioInitStructer );		//��ʼ��MISO
	
	//NSS����Ϊ�������
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_Out_PP;
	SpiGpioInitStructer.GPIO_Pin = SPI_NSS_GPIO_PIN;
	GPIO_Init( SPI_NSS_GPIO_PORT, &SpiGpioInitStructer );		//��ʼ��NSS
	GPIO_SetBits( SPI_NSS_GPIO_PORT, SPI_NSS_GPIO_PIN );		//�ø�

	/** SPI���� */
	SPI_I2S_DeInit( SPI_PORT );			//��λSPI
	
	if( SPI1 == SPI_PORT )				
	{
		RCC_APB2PeriphClockCmd( SPI_PORT_CLK, ENABLE );			//SPI1��APB2�ϣ�����ӦSPIʱ��
	}
	else
	{
		RCC_APB1PeriphClockCmd( SPI_PORT_CLK, ENABLE );			//SPI2 3��APB1��
	}
	
	SPI_Cmd( SPI_PORT, DISABLE );		//�ر�SPI���裬����ǰ�ر�
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//˫��ȫ˫��
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//����ģʽ
	SpiInitStructer.SPI_CPOL = SPI_CPOL_High;							//����״̬Ϊ�͵�ƽ 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_2Edge;							//��һ�����زɼ�����
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8λ����
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//�ӻ��������
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//2��Ƶ
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//���λ�ȷ���
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC����ʽ,Ĭ�ϲ�ʹ��SPI�Դ�CRC	 
	
	SPI_Init( SPI_PORT, &SpiInitStructer );
	SPI_Cmd( SPI_PORT, ENABLE );
}

/**
  * @brief :SPI�շ�һ���ֽ�
  * @param :
  *			@TxByte: ���͵������ֽ�
  * @note  :�Ƕ���ʽ��һ���ȴ���ʱ���������Զ��˳�
  * @retval:���յ����ֽ�
  */
uint8_t drv_spi_read_write_byte( uint8_t TxByte )
{
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_I2S_FLAG_TXE ) )		//�ȴ����ͻ�����Ϊ��
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//�������ý��յȴ�ʱ��(��ΪSPI���ٶȺܿ죬����������ڷ������֮��������յ����ݣ��ȴ�ʱ�䲻��Ҫ����)
	SPI_I2S_SendData(SPI_PORT, TxByte);	    //��������
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI_PORT, SPI_I2S_FLAG_RXNE ) )		//�ȴ����ջ������ǿ�
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	return SPI_I2S_ReceiveData(SPI_PORT);  //��������
	
}

/**
  * @brief :SPI���ַ���
  * @param :
  *			@WriteBuffer:�����ֽڻ�������ַ
  *			@Length:�ֽڳ���
  * @note  :�Ƕ���ʽ��һ���ȴ���ʱ���������Զ��˳�
  * @retval:��
  */
void drv_spi_transmit(uint8_t *WriteBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        drv_spi_read_write_byte(WriteBuffer[i]);
    }
}

/**
  * @brief :SPI���ַ���
  * @param :
  *			@ReadBuffer: �������ݻ�������ַ
  *			@Length:�ֽڳ���
  * @note  :�Ƕ���ʽ��һ���ȴ���ʱ���������Զ��˳�
  * @retval:��
  */
void drv_spi_receive(uint8_t *ReadBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        ReadBuffer[i] = drv_spi_read_write_byte(0);
    }
}
