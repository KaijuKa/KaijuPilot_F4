#include "iic.h"

/*******************************************************************************
* 函 数 名         : IIC_Init
* 函数功能		     : IIC初始化
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能 GPIOA 时钟
	
	//GPIOA5,A7初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能 GPIOB 时钟
//	
//	//GPIOB9,B8初始化设置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	IIC_SCL=1;
	IIC_SDA=1;	
}

/*******************************************************************************
* 函 数 名         : SDA_OUT
* 函数功能		     : SDA输出配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOA7初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
//	//GPIOB8初始化设置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//普通输出模式
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
}

/*******************************************************************************
* 函 数 名         : SDA_IN
* 函数功能		     : SDA输入配置	   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOA7初始化设置
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//输入模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
//	//GPIOB8初始化设置
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//输入模式
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
}

/*******************************************************************************
* 函 数 名         : IIC_Start
* 函数功能		     : 产生IIC起始信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(5);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	

/*******************************************************************************
* 函 数 名         : IIC_Stop
* 函数功能		     : 产生IIC停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_SCL=1; 
	delay_us(6); 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(6);							   	
}

/*******************************************************************************
* 函 数 名         : IIC_Wait_Ack
* 函数功能		     : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	delay_us(1);	   
	IIC_SCL=1;
	delay_us(1);	 
	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 

/*******************************************************************************
* 函 数 名         : IIC_Ack
* 函数功能		     : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(5);
	IIC_SCL=0;
}

/*******************************************************************************
* 函 数 名         : IIC_NAck
* 函数功能		     : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(5);
	IIC_SCL=0;
}	

/*******************************************************************************
* 函 数 名         : IIC_Send_Byte
* 函数功能		     : IIC发送一个字节 
* 输    入         : txd：发送一个字节
* 输    出         : 无
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	SDA_OUT(); 	    
  IIC_SCL=0;//拉低时钟开始数据传输
  for(t=0;t<8;t++)
  {              
		if((txd&0x80)>0) //0x80  1000 0000
			IIC_SDA=1;
		else
			IIC_SDA=0;
    txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
  }	 
} 

/*******************************************************************************
* 函 数 名         : IIC_Read_Byte
* 函数功能		     : IIC读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		IIC_SCL=0; 
    delay_us(2);
		IIC_SCL=1;
    receive<<=1;
    if(READ_SDA)receive++;   
		delay_us(1); 
  }					 
  if (!ack)
      IIC_NAck();//发送nACK
  else
      IIC_Ack(); //发送ACK   
  return receive;
}

/*******************************************************************************
* 函 数 名         : IIC_Read_Str
* 函数功能		     : IIC读多个字节
* 输    入         : 设备地址，寄存器地址，读取长度，读取容器
* 输    出         : 1 失败 0 成功
*******************************************************************************/
u8 IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	IIC_Start();                  //开始信号
	IIC_Send_Byte((addr<<1) | 0); //写设备地址 写
	if(IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	IIC_Send_Byte(reg);           //写寄存器地址
	if(IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	IIC_Start();                  //开始信号
	IIC_Send_Byte((addr<<1) | 1); //写设备地址 读
	if(IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	for(i = 0;i<len;i++)          //读取len个字节
	{
		if(i == len-1)              //最后一个字节 非应答
			buf[i] = IIC_Read_Byte(0);
		else               
			buf[i] = IIC_Read_Byte(1);
	}
	
	IIC_Stop();                   //终止信号
	return 0;
}

/*******************************************************************************
* 函 数 名         : IIC_Write_Str
* 函数功能		     : IIC写多个字节
* 输    入         : 设备地址，寄存器地址，读取长度，读取容器
* 输    出         : 1 失败 0 成功
*******************************************************************************/
u8 IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	IIC_Start();                  //开始信号
	IIC_Send_Byte((addr<<1) | 0); //写设备地址 写
	if(IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	IIC_Send_Byte(reg);           //写寄存器地址
	if(IIC_Wait_Ack() == 1)       //等待应答
		return 1;
	
	for(i = 0;i<len;i++)          //写len个字节
	{
														    //最后一个字节 非应答
		IIC_Send_Byte(buf[i]);
		if(IIC_Wait_Ack() == 1)     //等待应答
			return 1;
	}
	
	IIC_Stop();                   //终止信号
	return 0;
}
