#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx.h"


void DRV_SPI1_Init( void );
uint8_t DRV_SPI1_Read_Write_Byte( uint8_t TxByte );
void DRV_SPI1_Transmit(uint8_t *WriteBuffer, uint16_t Length);
void DRV_SPI1_Receive(uint8_t *ReadBuffer, uint16_t Length);

void DRV_SPI2_Init( void );
uint8_t DRV_SPI2_Read_Write_Byte( uint8_t TxByte );
void DRV_SPI2_Transmit(uint8_t *WriteBuffer, uint16_t Length);
void DRV_SPI2_Receive(uint8_t *ReadBuffer, uint16_t Length);

#endif
