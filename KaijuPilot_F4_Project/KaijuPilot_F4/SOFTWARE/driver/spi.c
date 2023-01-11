#include "spi.h"

/** Ӳ��SPI */
#define SPI_WAIT_TIMEOUT			((uint16_t)0xFFFF)

/*******************************************************************************
* �� �� ��         : DRV_SPI1_Init
* ��������		   : ��ʼ��spi1
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI1_Init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI�������� */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );	//�򿪶˿�ʱ��
	
	//SCK MOSI MISO ����Ϊ����
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF;
    SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_100MHz;
    SpiGpioInitStructer.GPIO_OType = GPIO_OType_PP;
    SpiGpioInitStructer.GPIO_PuPd  = GPIO_PuPd_UP;
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//��ʼ��SCK
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//��ʼ��MOSI
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//��ʼ��MISO
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	//NSS����Ϊ�������
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//��ʼ��NSS
	GPIO_SetBits( GPIOA, GPIO_Pin_4 );		        //�ø�

	/** SPI���� */
	SPI_I2S_DeInit( SPI1 );			//��λSPI
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );			//SPI1��APB2�ϣ�����ӦSPIʱ��
	
	SPI_Cmd( SPI1, DISABLE );		//�ر�SPI���裬����ǰ�ر�
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//˫��ȫ˫��
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//����ģʽ
	SpiInitStructer.SPI_CPOL = SPI_CPOL_High;							//����״̬Ϊ�ߵ�ƽ 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_2Edge;							//�ڶ������زɼ�����
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8λ����
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//�ӻ��������
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//2��Ƶ
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//���λ�ȷ���
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC����ʽ 
	
	SPI_Init( SPI1, &SpiInitStructer );
	SPI_Cmd( SPI1, ENABLE );
}

/*******************************************************************************
* �� �� ��         : DRV_SPI2_Init
* ��������		   : ��ʼ��spi2
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI2_Init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI�������� */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );	//�򿪶˿�ʱ��
	
	//SCK MOSI MISO ����Ϊ����
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF;
    SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_100MHz;
    SpiGpioInitStructer.GPIO_OType = GPIO_OType_PP;
    SpiGpioInitStructer.GPIO_PuPd  = GPIO_PuPd_UP;
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//��ʼ��SCK
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//��ʼ��MOSI
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//��ʼ��MISO
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	//NSS����Ϊ�������
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//��ʼ��NSS
	GPIO_SetBits( GPIOB, GPIO_Pin_12 );		        //�ø�

	/** SPI���� */
	SPI_I2S_DeInit( SPI2 );			//��λSPI
	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );			//SPI1��APB2�ϣ�����ӦSPIʱ��
	
	SPI_Cmd( SPI2, DISABLE );		//�ر�SPI���裬����ǰ�ر�
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//˫��ȫ˫��
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//����ģʽ
	SpiInitStructer.SPI_CPOL = SPI_CPOL_High;							//����״̬Ϊ�ߵ�ƽ 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_2Edge;							//�ڶ������زɼ�����
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8λ����
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//�ӻ��������
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//2��Ƶ
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//���λ�ȷ���
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC����ʽ 
	
	SPI_Init( SPI2, &SpiInitStructer );
	SPI_Cmd( SPI2, ENABLE );
}

/*******************************************************************************
* �� �� ��         : DRV_SPI1_Read_Write_Byte
* ��������		   : ����spi1��дһ���ֽ�����
* ��    ��         : д����ֽ�����
* ��    ��         : �������ֽ�����
*******************************************************************************/
uint8_t DRV_SPI1_Read_Write_Byte( uint8_t TxByte )
{
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) )		//�ȴ����ͻ�����Ϊ��
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//�������ý��յȴ�ʱ��(��ΪSPI���ٶȺܿ죬����������ڷ������֮��������յ����ݣ��ȴ�ʱ�䲻��Ҫ����)
	SPI_I2S_SendData(SPI1, TxByte);	    //��������
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) )		//�ȴ����ջ������ǿ�
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	return SPI_I2S_ReceiveData(SPI1);  //��������
	
}

/*******************************************************************************
* �� �� ��         : DRV_SPI1_Transmit
* ��������		   : ͨ��spi1д��һ������
* ��    ��         : ����buf ���ݳ���
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI1_Transmit(uint8_t *WriteBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        DRV_SPI1_Read_Write_Byte(WriteBuffer[i]);
    }
}

/*******************************************************************************
* �� �� ��         : DRV_SPI1_Receive
* ��������		   : ͨ��spi1����һ������
* ��    ��         : ����buf ���յ����ݳ���
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI1_Receive(uint8_t *ReadBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        ReadBuffer[i] = DRV_SPI1_Read_Write_Byte(0);
    }
}

/*******************************************************************************
* �� �� ��         : DRV_SPI2_Read_Write_Byte
* ��������		   : ����spi2��дһ���ֽ�����
* ��    ��         : д����ֽ�����
* ��    ��         : �������ֽ�����
*******************************************************************************/
uint8_t DRV_SPI2_Read_Write_Byte( uint8_t TxByte )
{
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_TXE ) )		//�ȴ����ͻ�����Ϊ��
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//�������ý��յȴ�ʱ��(��ΪSPI���ٶȺܿ죬����������ڷ������֮��������յ����ݣ��ȴ�ʱ�䲻��Ҫ����)
	SPI_I2S_SendData(SPI2, TxByte);	    //��������
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_RXNE ) )		//�ȴ����ջ������ǿ�
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//����ȴ���ʱ���˳�
		}
	}
	return SPI_I2S_ReceiveData(SPI2);  //��������
	
}

/*******************************************************************************
* �� �� ��         : DRV_SPI2_Transmit
* ��������		   : ͨ��spi2д��һ������
* ��    ��         : ����buf ���ݳ���
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI2_Transmit(uint8_t *WriteBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        DRV_SPI2_Read_Write_Byte(WriteBuffer[i]);
    }
}

/*******************************************************************************
* �� �� ��         : DRV_SPI2_Receive
* ��������		   : ͨ��spi2����һ������
* ��    ��         : ����buf ���յ����ݳ���
* ��    ��         : ��
*******************************************************************************/
void DRV_SPI2_Receive(uint8_t *ReadBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        ReadBuffer[i] = DRV_SPI2_Read_Write_Byte(0);
    }
}

