#include "uart.h"
#include "remote_signal.h"

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
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
* �� �� ��         : uart1_init
* ��������		   : ��ʼ��uart1
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void uart1_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1, ENABLE);	//ʹ��USART1��GPIOAʱ��
	
	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
    //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 
}

u8 TxBuffer1[256];
u8 TxCounter1 = 0;
u8 count1 = 0;
u8 Rx_data = 0;

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		Rx_data = USART_ReceiveData(USART1);//(USART2->DR);	//��ȡ���յ�������
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
* �� �� ��         : Uart1_Send
* ��������		     : uart1�������� 	   
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
		USART_ITConfig ( USART1, USART_IT_TXE, ENABLE ); //���������ж�
	}
}

/*******************************************************************************
* �� �� ��         : uart2_init
* ��������		   : ��ʼ��uart2
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void uart2_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	//USART2TX   GPIOa.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //Pa.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOa.2
   
  //USART2_RX	  GPIOa.3��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//Pa3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOa.3  

  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2
}

u8 TxBuffer2[256];
u8 TxCounter2 = 0;
u8 count2 = 0;

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 com_data;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART2);//(USART2->DR);	//��ȡ���յ�������
	}
	
	if (USART_GetITStatus(USART2,USART_IT_TXE))
	{
		USART2->DR = TxBuffer2[TxCounter2++]; 
		if (TxCounter2==count2)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//�ر��ж�
		}
	}
} 

/*******************************************************************************
* �� �� ��         : Uart2_Send
* ��������		     : uart2�������� 	   
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
    USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //���������ж�
  }
}

/*******************************************************************************
* �� �� ��         : uart3_init
* ��������		   : ��ʼ��uart3
* ��    ��         : ������
* ��    ��         : ��
*******************************************************************************/
void uart3_init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART3��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	//USART3TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB.10
   
  //USART3_RX	  GPIOB.11��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB11  

  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_Even;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ

  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3
}

void USART3_IRQHandler(void)                	//����2�жϷ������
{
	u8 com_data;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		com_data =USART_ReceiveData(USART3);//(USART2->DR);	//��ȡ���յ�������
		Channel_data_ByteGet(com_data);
	}
} 

/*******************************************************************************
* �� �� ��         : Uart3_Send
* ��������		     : uart3�������� 
* ��    ��         : ����buffer ������
* ��    ��         : ��
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
