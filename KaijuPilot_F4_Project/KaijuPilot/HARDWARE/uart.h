#ifndef __UART_H__
#define __UART_H__


#include "io.h"
#include "stdio.h"

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			0		//ʹ�ܣ�1��/��ֹ��0������1����

	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern u8 Rx_data;

//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void Uart1_Send(u8 *databuffer,u8 data_num);
void Uart2_Send(u8 *databuffer,u8 data_num);
void Uart3_Send(u8 *databuffer,u8 data_num);
void OPMV_ByteGet(u8 data);
void OPMV_Analysis(void);
void OPMV_offline_reset(void);
void OPMV_offline_check(u8 dT_ms);

#endif
