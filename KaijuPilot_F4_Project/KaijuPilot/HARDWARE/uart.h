#ifndef __UART_H__
#define __UART_H__


#include "io.h"
#include "stdio.h"

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			0		//使能（1）/禁止（0）串口1接收

	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern u8 Rx_data;

//如果想串口中断接收，请不要注释以下宏定义
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
