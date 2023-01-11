#include "spi.h"

/** 硬件SPI */
#define SPI_WAIT_TIMEOUT			((uint16_t)0xFFFF)

/*******************************************************************************
* 函 数 名         : DRV_SPI1_Init
* 函数功能		   : 初始化spi1
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_SPI1_Init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI引脚配置 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );	//打开端口时钟
	
	//SCK MOSI MISO 配置为复用
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF;
    SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_100MHz;
    SpiGpioInitStructer.GPIO_OType = GPIO_OType_PP;
    SpiGpioInitStructer.GPIO_PuPd  = GPIO_PuPd_UP;
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//初始化SCK
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//初始化MOSI
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//初始化MISO
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	//NSS配置为推挽输出
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init( GPIOA, &SpiGpioInitStructer );		//初始化NSS
	GPIO_SetBits( GPIOA, GPIO_Pin_4 );		        //置高

	/** SPI配置 */
	SPI_I2S_DeInit( SPI1 );			//复位SPI
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE );			//SPI1在APB2上，打开相应SPI时钟
	
	SPI_Cmd( SPI1, DISABLE );		//关闭SPI外设，配置前关闭
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线全双工
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//主机模式
	SpiInitStructer.SPI_CPOL = SPI_CPOL_High;							//空闲状态为高电平 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_2Edge;							//第二个边沿采集数据
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8位数据
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//从机软件管理
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//2分频
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//最高位先发送
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC多项式 
	
	SPI_Init( SPI1, &SpiInitStructer );
	SPI_Cmd( SPI1, ENABLE );
}

/*******************************************************************************
* 函 数 名         : DRV_SPI2_Init
* 函数功能		   : 初始化spi2
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_SPI2_Init( void )
{
	GPIO_InitTypeDef	SpiGpioInitStructer;
	SPI_InitTypeDef		SpiInitStructer;
	
	/** SPI引脚配置 */
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );	//打开端口时钟
	
	//SCK MOSI MISO 配置为复用
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_AF;
    SpiGpioInitStructer.GPIO_Speed = GPIO_Speed_100MHz;
    SpiGpioInitStructer.GPIO_OType = GPIO_OType_PP;
    SpiGpioInitStructer.GPIO_PuPd  = GPIO_PuPd_UP;
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//初始化SCK
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//初始化MOSI
	
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//初始化MISO
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	//NSS配置为推挽输出
	SpiGpioInitStructer.GPIO_Mode = GPIO_Mode_OUT;
	SpiGpioInitStructer.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init( GPIOB, &SpiGpioInitStructer );		//初始化NSS
	GPIO_SetBits( GPIOB, GPIO_Pin_12 );		        //置高

	/** SPI配置 */
	SPI_I2S_DeInit( SPI2 );			//复位SPI
	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );			//SPI1在APB2上，打开相应SPI时钟
	
	SPI_Cmd( SPI2, DISABLE );		//关闭SPI外设，配置前关闭
	
	SpiInitStructer.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线全双工
	SpiInitStructer.SPI_Mode = SPI_Mode_Master;							//主机模式
	SpiInitStructer.SPI_CPOL = SPI_CPOL_High;							//空闲状态为高电平 
	SpiInitStructer.SPI_CPHA = SPI_CPHA_2Edge;							//第二个边沿采集数据
	SpiInitStructer.SPI_DataSize = SPI_DataSize_8b;						//8位数据
	SpiInitStructer.SPI_NSS = SPI_NSS_Soft;								//从机软件管理
	SpiInitStructer.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//2分频
	SpiInitStructer.SPI_FirstBit = SPI_FirstBit_MSB;					//最高位先发送
	SpiInitStructer.SPI_CRCPolynomial = 7;								//CRC多项式 
	
	SPI_Init( SPI2, &SpiInitStructer );
	SPI_Cmd( SPI2, ENABLE );
}

/*******************************************************************************
* 函 数 名         : DRV_SPI1_Read_Write_Byte
* 函数功能		   : 驱动spi1读写一个字节数据
* 输    入         : 写入的字节数据
* 输    出         : 读出的字节数据
*******************************************************************************/
uint8_t DRV_SPI1_Read_Write_Byte( uint8_t TxByte )
{
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) )		//等待发送缓冲区为空
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//重新设置接收等待时间(因为SPI的速度很快，正常情况下在发送完成之后会立即收到数据，等待时间不需要过长)
	SPI_I2S_SendData(SPI1, TxByte);	    //发送数据
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_RXNE ) )		//等待接收缓冲区非空
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	return SPI_I2S_ReceiveData(SPI1);  //接收数据
	
}

/*******************************************************************************
* 函 数 名         : DRV_SPI1_Transmit
* 函数功能		   : 通过spi1写入一串数据
* 输    入         : 数据buf 数据长度
* 输    出         : 无
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
* 函 数 名         : DRV_SPI1_Receive
* 函数功能		   : 通过spi1接收一串数据
* 输    入         : 数据buf 接收的数据长度
* 输    出         : 无
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
* 函 数 名         : DRV_SPI2_Read_Write_Byte
* 函数功能		   : 驱动spi2读写一个字节数据
* 输    入         : 写入的字节数据
* 输    出         : 读出的字节数据
*******************************************************************************/
uint8_t DRV_SPI2_Read_Write_Byte( uint8_t TxByte )
{
	uint16_t l_WaitTime = 0;
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_TXE ) )		//等待发送缓冲区为空
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	l_WaitTime = SPI_WAIT_TIMEOUT / 2;		//重新设置接收等待时间(因为SPI的速度很快，正常情况下在发送完成之后会立即收到数据，等待时间不需要过长)
	SPI_I2S_SendData(SPI2, TxByte);	    //发送数据
	
	while( RESET == SPI_I2S_GetFlagStatus( SPI2, SPI_I2S_FLAG_RXNE ) )		//等待接收缓冲区非空
	{
		if( SPI_WAIT_TIMEOUT == ++l_WaitTime )
		{
			break;			//如果等待超时则退出
		}
	}
	return SPI_I2S_ReceiveData(SPI2);  //接收数据
	
}

/*******************************************************************************
* 函 数 名         : DRV_SPI2_Transmit
* 函数功能		   : 通过spi2写入一串数据
* 输    入         : 数据buf 数据长度
* 输    出         : 无
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
* 函 数 名         : DRV_SPI2_Receive
* 函数功能		   : 通过spi2接收一串数据
* 输    入         : 数据buf 接收的数据长度
* 输    出         : 无
*******************************************************************************/
void DRV_SPI2_Receive(uint8_t *ReadBuffer, uint16_t Length)
{
	uint16_t i=0;
    for(i=0; i<Length; i++)
    {
        ReadBuffer[i] = DRV_SPI2_Read_Write_Byte(0);
    }
}

