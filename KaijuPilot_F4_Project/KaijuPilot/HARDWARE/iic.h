#ifndef  __IIC_H__
#define  __IIC_H__

#include "io.h"
#include "delay.h"

//IO操作函数	 
#define IIC_SCL    PAout(5) //SCL
#define IIC_SDA    PAout(7) //SDA	 
#define READ_SDA   PAin(7)  //输入SDA

//#define IIC_SCL    PBout(9) //SCL
//#define IIC_SDA    PBout(8) //SDA	 
//#define READ_SDA   PBin(8)  //输入SDA

void IIC_Init(void);
void SDA_OUT(void);
void SDA_IN(void);
void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(u8 ack);
u8 IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf);
u8 IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf);

#endif
