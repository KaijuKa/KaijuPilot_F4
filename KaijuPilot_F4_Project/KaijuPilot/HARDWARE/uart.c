#include "uart.h"
#include "remote_signal.h"

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
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
* 函 数 名         : uart1_init
* 函数功能		   : 初始化uart1
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void uart1_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);	//使能USART1，GPIOA时钟
	
	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
    //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                    //使能串口1 
}

u8 TxBuffer1[256];
u8 TxCounter1 = 0;
u8 count1 = 0;
u8 Rx_data = 0;

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		Rx_data = USART_ReceiveData(USART1);//(USART2->DR);	//读取接收到的数据
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
* 函 数 名         : Uart1_Send
* 函数功能		     : uart1发送数据 	   
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void Uart1_Send(u8 *databuffer,u8 data_num)
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
* 函 数 名         : uart2_init
* 函数功能		   : 初始化uart2
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void uart2_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//USART2TX   GPIOa.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //Pa.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOa.2
   
  //USART2_RX	  GPIOa.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//Pa3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOa.3  

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2
}

u8 TxBuffer2[256];
u8 TxCounter2 = 0;
u8 count2 = 0;

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 com_data;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART2);//(USART2->DR);	//读取接收到的数据
	}
	
	if (USART_GetITStatus(USART2,USART_IT_TXE))
	{
		USART2->DR = TxBuffer2[TxCounter2++]; 
		if (TxCounter2==count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭中断
		}
	}
} 

/*******************************************************************************
* 函 数 名         : Uart2_Send
* 函数功能		     : uart2发送数据 	   
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void Uart2_Send(u8 *databuffer,u8 data_num)
{
	u8 i;
	for(i = 0;i < data_num;i++)
	{
		TxBuffer2[count2++] = * ( databuffer + i );
	}
	if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) )
  {
    USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //开启发送中断
  }
}

/*******************************************************************************
* 函 数 名         : uart3_init
* 函数功能		   : 初始化uart3
* 输    入         : 波特率
* 输    出         : 无
*******************************************************************************/
void uart3_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART3，GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//USART3TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10
   
  //USART3_RX	  GPIOB.11初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB11  

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//两个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3
}

void USART3_IRQHandler(void)                	//串口2中断服务程序
{
	u8 com_data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART3);//(USART2->DR);	//读取接收到的数据
		Channel_data_ByteGet(com_data);
	}
} 

/*******************************************************************************
* 函 数 名         : Uart3_Send
* 函数功能		     : uart3发送数据 
* 输    入         : 数据buffer 数据量
* 输    出         : 无
*******************************************************************************/
void Uart3_Send(u8 *databuffer,u8 data_num)
{
	u8 i = 0;
	while(data_num--)
	{
		USART_SendData(USART3,databuffer[i]);	
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC) != SET);
		USART_ClearFlag(USART3,USART_FLAG_TC);
		i++;
	}
}
