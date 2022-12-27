#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx.h"

//SPI引脚定义
#define SPI_CLK_GPIO_PORT			GPIOA
#define SPI_CLK_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_CLK_GPIO_PIN			GPIO_Pin_5

#define SPI_MISO_GPIO_PORT			GPIOA
#define SPI_MISO_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_MISO_GPIO_PIN			GPIO_Pin_6

#define SPI_MOSI_GPIO_PORT			GPIOA
#define SPI_MOSI_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_MOSI_GPIO_PIN			GPIO_Pin_7

#define SPI_NSS_GPIO_PORT			GPIOA
#define SPI_NSS_GPIO_CLK			RCC_AHB1Periph_GPIOA
#define SPI_NSS_GPIO_PIN			GPIO_Pin_4


#define spi_set_nss_high( )			SPI_NSS_GPIO_PORT->ODR |= SPI_NSS_GPIO_PIN								//片选置高
#define spi_set_nss_low( )			SPI_NSS_GPIO_PORT->ODR &= (uint32_t)( ~((uint32_t)SPI_NSS_GPIO_PIN ))	//片选置低


//SPI接口定义
#define SPI_PORT					SPI1						//SPI接口
#define SPI_PORT_CLK				RCC_APB2Periph_SPI1			//SPI时钟


void DRV_SPI_Init( void );
uint8_t DRV_SPI_Read_Write_Byte( uint8_t TxByte );
void DRV_SPI_Transmit(uint8_t *WriteBuffer, uint16_t Length);
void DRV_SPI_Receive(uint8_t *ReadBuffer, uint16_t Length);

#endif
