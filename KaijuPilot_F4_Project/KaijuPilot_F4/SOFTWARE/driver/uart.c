#include "uart.h"
#include "remote_signal.h"
#include "msg_interchange.h"
#include "GPS.h"

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


/*******************************************************************************
* �� �� ��         : DRV_USART1_Init
* ��������		   : ��ʼ��usart1
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void DRV_USART1_Init(u32 bound){
	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	//����ʱ��
    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_USART1, ENABLE ); 
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );

    //�����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );

    //����PA9��ΪUSART1��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //����PA10��ΪUSART1��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
	
	GPIO_PinAFConfig ( GPIOA, GPIO_PinSource9, GPIO_AF_USART1 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource10, GPIO_AF_USART1 );

    //����USART1
    //�жϱ�������
    USART_InitStructure.USART_BaudRate = bound;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
    //����USART1ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init ( USART1, &USART_InitStructure );
    USART_ClockInit ( USART1, &USART_ClockInitStruct );

    //ʹ��USART1�����ж�
    USART_ITConfig ( USART1, USART_IT_RXNE, ENABLE );
    //ʹ��USART1
    USART_Cmd ( USART1, ENABLE );
}

u8 TxBuffer1[256];
u8 TxCounter1 = 0;
u8 count1 = 0;
u8 Rx_data = 0;

void USART1_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		com_data = USART_ReceiveData(USART1); //��ȡ���յ�������
		MSG_RECV_ByteGet(com_data);
	}
	
	if (USART_GetITStatus(USART1,USART_IT_TXE))
	{
		USART1->DR = TxBuffer1[TxCounter1++]; 
		if (TxCounter1==count1)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//�ر��ж�
		}
	}
} 

/*******************************************************************************
* �� �� ��         : DRV_USART1_Send
* ��������		     :USART1�������� 	   
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
		USART_ITConfig ( USART1, USART_IT_TXE, ENABLE ); //���������ж�
	}
}

/*******************************************************************************
* �� �� ��         : DRV_USART3_Init
* ��������		   : ��ʼ��USART3
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void DRV_USART3_Init(u32 bound)
{
	USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	//����ʱ��
    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE ); 
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //�����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );

    //����PD8��ΪUSART3��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
    //����PD9��ΪUSART3��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );
	
	GPIO_PinAFConfig ( GPIOD, GPIO_PinSource8, GPIO_AF_USART3 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource9, GPIO_AF_USART3 );

    //����USART3
    //�жϱ�������
    USART_InitStructure.USART_BaudRate = bound;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //��֡��β����1��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;    //������żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //���͡�����ʹ��
    //����USART3ʱ��
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //ʱ�ӵ͵�ƽ�
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK������ʱ������ļ���->�͵�ƽ
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //ʱ�ӵڶ������ؽ������ݲ���
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //���һλ���ݵ�ʱ�����岻��SCLK���

    USART_Init ( USART3, &USART_InitStructure );
    USART_ClockInit ( USART3, &USART_ClockInitStruct );

    //ʹ��USART3�����ж�
    USART_ITConfig ( USART3, USART_IT_RXNE, ENABLE );
    //ʹ��USART3
    USART_Cmd ( USART3, ENABLE );
}

u8 TxBuffer2[256];
u8 TxCounter2 = 0;
u8 count2 = 0;

void USART3_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART3);	//��ȡ���յ�������
		DRV_GPS_ByteGet(com_data);
	}
	
	if (USART_GetITStatus(USART3,USART_IT_TXE))
	{
		USART3->DR = TxBuffer2[TxCounter2++]; 
		if (TxCounter2==count2)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر��ж�
		}
	}
} 

/*******************************************************************************
* �� �� ��         : DRV_USART3_Send
* ��������		     :USART3�������� 	   
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
		USART_ITConfig ( USART3, USART_IT_TXE, ENABLE ); //���������ж�
	}
}

/*******************************************************************************
* �� �� ��         : DRV_UART5_Init
* ��������		   : ��ʼ��uart5
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void DRV_UART5_Init(u32 bound)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5, ENABLE ); //����USART2ʱ��
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );

    //�����ж����ȼ�
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );

    //����PC12��ΪUART5��Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );
    //����PD2��ΪUART5��Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOD, &GPIO_InitStructure );

    //����UART5
    //�жϱ�������
    USART_InitStructure.USART_BaudRate = bound;       
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8λ����
    USART_InitStructure.USART_StopBits = USART_StopBits_2;   //��֡��β����2��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_Even;    //żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //Ӳ��������ʧ��
    USART_InitStructure.USART_Mode = USART_Mode_Rx;  //���͡�����ʹ��
    USART_Init ( UART5, &USART_InitStructure );

    //ʹ��UART5�����ж�
    USART_ITConfig ( UART5, USART_IT_RXNE, ENABLE );
    //ʹ��UART5
    USART_Cmd ( UART5, ENABLE );
}

void UART5_IRQHandler(void)                	
{
	u8 com_data;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		com_data =USART_ReceiveData(UART5); //��ȡ���յ�������
		RC_SBUS_ByteGet(com_data);
	}
} 

/*******************************************************************************
* �� �� ��         : DRV_UART5_Send
* ��������		     : uart5�������� 
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
