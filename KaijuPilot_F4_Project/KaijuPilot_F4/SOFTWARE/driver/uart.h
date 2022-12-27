#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx.h"
#include "stdio.h"

extern u8 Rx_data;

void DRV_USART1_Init(u32 bound);
void DRV_USART3_Init(u32 bound);
void DRV_UART5_Init(u32 bound);
void DRV_USART1_Send(u8 *databuffer,u8 data_num);
void DRV_USART3_Send(u8 *databuffer,u8 data_num);
void DRV_UART5_Send(u8 *databuffer,u8 data_num);

#endif
