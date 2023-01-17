#include "uart.h"
#include "remote_signal.h"
#include "msg_interchange.h"
#include "GPS.h"

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


/*******************************************************************************
* 函 数 名         : DRV_USART1_Init
* 函数功能		   : 初始化usart1
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void DRV_USART1_Init(u32 bound){
	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	//开启时钟
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_USART1, ENABLE ); 
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );

    //配置PA9作为USART1　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //配置PA10作为USART1　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
	
	GPIO_PinAFConfig ( GPIOA, GPIO_PinSource9, GPIO_AF_USART1 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource10, GPIO_AF_USART1 );

    //配置USART1
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = bound;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART1时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART1, &USART_InitStructure );
    USART_ClockInit ( USART1, &USART_ClockInitStruct );

    //使能USART1接收中断
    USART_ITConfig ( USART1, USART_IT_RXNE, ENABLE );
    //使能USART1
    USART_Cmd ( USART1, ENABLE );
}

u8 TxBuffer1[256];
u8 TxCounter1 = 0;
u8 count1 = 0;
u8 Rx_data = 0;

void USART1_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		com_data = USART_ReceiveData(USART1); //读取接收到的数据
		MSG_RECV_ByteGet(com_data);
	}
	
	if (USART_GetITStatus(USART1,USART_IT_TXE))
	{
		USART1->DR = TxBuffer1[TxCounter1++]; 
		if (TxCounter1==count1)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭中断
		}
	}
} 

/*******************************************************************************
* 函 数 名         : DRV_USART1_Send
* 函数功能		     :USART1发送数据 	   
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void DRV_USART1_Send(u8 *databuffer,u8 data_num)
{
	u8 i;
	for(i = 0;i < data_num;i++)
	{
		TxBuffer1[count1++] = * ( databuffer + i );
	}
	if ( ! ( USART1->CR1 & USART_CR1_TXEIE ) )
	{
		USART_ITConfig ( USART1, USART_IT_TXE, ENABLE ); //开启发送中断
	}
}

/*******************************************************************************
* 函 数 名         : DRV_USART3_Init
* 函数功能		   : 初始化USART3
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void DRV_USART3_Init(u32 bound)
{
	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	//开启时钟
    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE ); 
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );

    //配置PD8作为USART3　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
    //配置PD9作为USART3　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
	
	GPIO_PinAFConfig ( GPIOD, GPIO_PinSource8, GPIO_AF_USART3 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource9, GPIO_AF_USART3 );

    //配置USART3
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = bound;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART3时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART3, &USART_InitStructure );
    USART_ClockInit ( USART3, &USART_ClockInitStruct );

    //使能USART3接收中断
    USART_ITConfig ( USART3, USART_IT_RXNE, ENABLE );
    //使能USART3
    USART_Cmd ( USART3, ENABLE );
}

u8 TxBuffer2[256];
u8 TxCounter2 = 0;
u8 count2 = 0;

void USART3_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART3);	//读取接收到的数据
		DRV_GPS_ByteGet(com_data);
	}
	
	if (USART_GetITStatus(USART3,USART_IT_TXE))
	{
		USART3->DR = TxBuffer2[TxCounter2++]; 
		if (TxCounter2==count2)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭中断
		}
	}
} 

/*******************************************************************************
* 函 数 名         : DRV_USART3_Send
* 函数功能		     :USART3发送数据 	   
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void DRV_USART3_Send(u8 *databuffer,u8 data_num)
{
	u8 i;
	for(i = 0;i < data_num;i++)
	{
		TxBuffer2[count2++] = * ( databuffer + i );
	}
	if ( ! ( USART3->CR1 & USART_CR1_TXEIE ) )
	{
		USART_ITConfig ( USART3, USART_IT_TXE, ENABLE ); //开启发送中断
	}
}

/*******************************************************************************
* 函 数 名         : DRV_UART5_Init
* 函数功能		   : 初始化uart5
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void DRV_UART5_Init(u32 bound)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

    //配置PC12作为UART5　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
    //配置PD2作为UART5　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //配置UART5
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = bound;       
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_2;   //在帧结尾传输2个停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even;    //偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Rx;  //发送、接收使能
    USART_Init ( UART5, &USART_InitStructure );

    //使能UART5接收中断
    USART_ITConfig ( UART5, USART_IT_RXNE, ENABLE );
    //使能UART5
    USART_Cmd ( UART5, ENABLE );
}

void UART5_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		com_data =USART_ReceiveData(UART5); //读取接收到的数据
		RC_SBUS_ByteGet(com_data);
	}
} 

/*******************************************************************************
* 函 数 名         : DRV_UART5_Send
* 函数功能		     : uart5发送数据 
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void DRV_UART5_Send(u8 *databuffer,u8 data_num)
{
	u8 i = 0;
	while(data_num--)
	{
		USART_SendData(UART5,databuffer[i]);	
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC) != SET);
		USART_ClearFlag(UART5,USART_FLAG_TC);
		i++;
	}
}
