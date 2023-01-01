#ifndef  __IIC_H__
#define  __IIC_H__

#include "stm32f4xx.h"
#include "delay.h"

//IO²Ù×÷º¯Êý	 
#define IIC_SCL_HIGH  GPIO_SetBits(GPIOC, GPIO_Pin_7)
#define IIC_SCL_LOW   GPIO_ResetBits(GPIOC, GPIO_Pin_7)
#define IIC_SDA_HIGH  GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define IIC_SDA_LOW   GPIO_ResetBits(GPIOC, GPIO_Pin_8)

#define READ_SDA      GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)

void DRV_IIC_Init(void);
void DRV_SDA_OUT(void);
void DRV_SDA_IN(void);
void DRV_IIC_Start(void);
void DRV_IIC_Stop(void);
u8 DRV_IIC_Wait_Ack(void);
void DRV_IIC_Ack(void);
void DRV_IIC_NAck(void);
u8 DRV_IIC_Send_Byte(u8 txd);
u8 DRV_IIC_Read_Byte(u8 ack);
u8 DRV_IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf);
u8 DRV_IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf);

#endif
