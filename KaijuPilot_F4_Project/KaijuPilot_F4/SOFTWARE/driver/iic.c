#include "iic.h"

/*******************************************************************************
* 函 数 名         : DRV_IIC_Init
* 函数功能		     : IIC初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能 GPIOC 时钟
	
	//GPIOC7,C8初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOC, &GPIO_InitStructure );

	IIC_SCL_HIGH;
	IIC_SDA_HIGH;	
}

/*******************************************************************************
* 函 数 名         : DRV_DRV_SDA_OUT
* 函数功能		     : SDA输出配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOC8初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
}

/*******************************************************************************
* 函 数 名         : DRV_DRV_SDA_IN
* 函数功能		     : SDA输入配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOC8初始化设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
}

/*******************************************************************************
* 函 数 名         : DRV_DRV_IIC_Start
* 函数功能		     : 产生IIC起始信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_IIC_Start(void)
{
	DRV_SDA_OUT();     //sda线输出
	IIC_SDA_HIGH;	  	  
	IIC_SCL_HIGH;
	delay_us(5);
 	IIC_SDA_LOW;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	IIC_SCL_LOW;//钳住I2C总线，准备发送或接收数据 
}

/*******************************************************************************
* 函 数 名         : DRV_IIC_Stop
* 函数功能		     : 产生IIC停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_IIC_Stop(void)
{
	DRV_SDA_OUT();//sda线输出
	IIC_SCL_LOW;
	IIC_SDA_LOW;//STOP:when CLK is high DATA change form low to high
 	IIC_SCL_HIGH; 
	delay_us(6); 
	IIC_SDA_HIGH;//发送I2C总线结束信号
	delay_us(6);							   	
}

/*******************************************************************************
* 函 数 名         : DRV_IIC_Wait_Ack
* 函数功能		     : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 DRV_IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	DRV_SDA_IN();      //SDA设置为输入  
	IIC_SDA_HIGH;
	delay_us(1);	   
	IIC_SCL_HIGH;
	delay_us(1);	 
	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			DRV_IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_LOW;//时钟输出0 	   
	return 0;  
} 

/*******************************************************************************
* 函 数 名         : DRV_IIC_Ack
* 函数功能		     : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void DRV_IIC_Ack(void)
{
	IIC_SCL_LOW;
	DRV_SDA_OUT();
	IIC_SDA_LOW;
	delay_us(2);
	IIC_SCL_HIGH;
	delay_us(5);
	IIC_SCL_LOW;
}

/*******************************************************************************
* 函 数 名         : DRV_IIC_NAck
* 函数功能		     : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/		    
void DRV_IIC_NAck(void)
{
	IIC_SCL_LOW;
	DRV_SDA_OUT();
	IIC_SDA_HIGH;
	delay_us(2);
	IIC_SCL_HIGH;
	delay_us(5);
	IIC_SCL_LOW;
}	

/*******************************************************************************
* 函 数 名         : DRV_IIC_Send_Byte
* 函数功能		     : IIC发送一个字节 
* 输    入         : txd：发送一个字节
* 输    出         : 0发送成功 1发送失败
*******************************************************************************/		  
u8 DRV_IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	DRV_SDA_OUT(); 	    
	IIC_SCL_LOW;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		if((txd&0x80)>0) //0x80  1000 0000
			IIC_SDA_HIGH;
		else
			IIC_SDA_LOW;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_HIGH;
		delay_us(2); 
		IIC_SCL_LOW;	
		delay_us(2);
	}	
	return DRV_IIC_Wait_Ack();
} 

/*******************************************************************************
* 函 数 名         : DRV_IIC_Read_Byte
* 函数功能		     : IIC读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/  
u8 DRV_IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	DRV_SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		IIC_SCL_LOW; 
		delay_us(2);
		IIC_SCL_HIGH;
		receive<<=1;
		if(READ_SDA)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
		DRV_IIC_NAck();//发送nACK
	else
		DRV_IIC_Ack(); //发送ACK   
	return receive;
}

/*******************************************************************************
* 函 数 名         : DRV_IIC_Read_Str
* 函数功能		     : IIC读多个字节
* 输    入         : 设备地址，寄存器地址，读取长度，读取容器
* 输    出         : 1 失败 0 成功
*******************************************************************************/
u8 DRV_IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	DRV_IIC_Start();                  //开始信号
	DRV_IIC_Send_Byte((addr<<1) | 0); //写设备地址 写
	if(DRV_IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	DRV_IIC_Send_Byte(reg);           //写寄存器地址
	if(DRV_IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	DRV_IIC_Start();                  //开始信号
	DRV_IIC_Send_Byte((addr<<1) | 1); //写设备地址 读
	if(DRV_IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	for(i = 0;i<len;i++)          //读取len个字节
	{
		if(i == len-1)              //最后一个字节 非应答
			buf[i] = DRV_IIC_Read_Byte(0);
		else               
			buf[i] = DRV_IIC_Read_Byte(1);
	}
	
	DRV_IIC_Stop();                   //终止信号
	return 0;
}

/*******************************************************************************
* 函 数 名         : DRV_IIC_Write_Str
* 函数功能		     : IIC写多个字节
* 输    入         : 设备地址，寄存器地址，读取长度，读取容器
* 输    出         : 1 失败 0 成功
*******************************************************************************/
u8 DRV_IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	DRV_IIC_Start();                  //开始信号
	DRV_IIC_Send_Byte((addr<<1) | 0); //写设备地址 写
	if(DRV_IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	DRV_IIC_Send_Byte(reg);           //写寄存器地址
	if(DRV_IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	for(i = 0;i<len;i++)          //写len个字节
	{
														    //最后一个字节 非应答
		DRV_IIC_Send_Byte(buf[i]);
		if(DRV_IIC_Wait_Ack() == 1)     //等待应答
			return 1;
	}
	
	DRV_IIC_Stop();                   //终止信号
	return 0;
}
