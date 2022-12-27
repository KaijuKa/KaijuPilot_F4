#include "iic.h"

/*******************************************************************************
* �� �� ��         : IIC_Init
* ��������		     : IIC��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ�� GPIOA ʱ��
	
	//GPIOA5,A7��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ�� GPIOB ʱ��
//	
//	//GPIOB9,B8��ʼ������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
	
	IIC_SCL=1;
	IIC_SDA=1;	
}

/*******************************************************************************
* �� �� ��         : SDA_OUT
* ��������		     : SDA�������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOA7��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
//	//GPIOB8��ʼ������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��ͨ���ģʽ
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

/*******************************************************************************
* �� �� ��         : SDA_IN
* ��������		     : SDA��������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOA7��ʼ������
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����ģʽ
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
//	//GPIOB8��ʼ������
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//����ģʽ
//	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
}

/*******************************************************************************
* �� �� ��         : IIC_Start
* ��������		     : ����IIC��ʼ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(5);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

/*******************************************************************************
* �� �� ��         : IIC_Stop
* ��������		     : ����IICֹͣ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	IIC_SCL=1; 
	delay_us(6); 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(6);							   	
}

/*******************************************************************************
* �� �� ��         : IIC_Wait_Ack
* ��������		     : �ȴ�Ӧ���źŵ���   
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 

/*******************************************************************************
* �� �� ��         : IIC_Ack
* ��������		     : ����ACKӦ��  
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : IIC_NAck
* ��������		     : ����NACK��Ӧ��  
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : IIC_Send_Byte
* ��������		     : IIC����һ���ֽ� 
* ��    ��         : txd������һ���ֽ�
* ��    ��         : ��
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	SDA_OUT(); 	    
  IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {              
		if((txd&0x80)>0) //0x80  1000 0000
			IIC_SDA=1;
		else
			IIC_SDA=0;
    txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
  }	 
} 

/*******************************************************************************
* �� �� ��         : IIC_Read_Byte
* ��������		     : IIC��һ���ֽ� 
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK 
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/  
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
      IIC_NAck();//����nACK
  else
      IIC_Ack(); //����ACK   
  return receive;
}

/*******************************************************************************
* �� �� ��         : IIC_Read_Str
* ��������		     : IIC������ֽ�
* ��    ��         : �豸��ַ���Ĵ�����ַ����ȡ���ȣ���ȡ����
* ��    ��         : 1 ʧ�� 0 �ɹ�
*******************************************************************************/
u8 IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	IIC_Start();                  //��ʼ�ź�
	IIC_Send_Byte((addr<<1) | 0); //д�豸��ַ д
	if(IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	IIC_Send_Byte(reg);           //д�Ĵ�����ַ
	if(IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	IIC_Start();                  //��ʼ�ź�
	IIC_Send_Byte((addr<<1) | 1); //д�豸��ַ ��
	if(IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	for(i = 0;i<len;i++)          //��ȡlen���ֽ�
	{
		if(i == len-1)              //���һ���ֽ� ��Ӧ��
			buf[i] = IIC_Read_Byte(0);
		else               
			buf[i] = IIC_Read_Byte(1);
	}
	
	IIC_Stop();                   //��ֹ�ź�
	return 0;
}

/*******************************************************************************
* �� �� ��         : IIC_Write_Str
* ��������		     : IICд����ֽ�
* ��    ��         : �豸��ַ���Ĵ�����ַ����ȡ���ȣ���ȡ����
* ��    ��         : 1 ʧ�� 0 �ɹ�
*******************************************************************************/
u8 IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	IIC_Start();                  //��ʼ�ź�
	IIC_Send_Byte((addr<<1) | 0); //д�豸��ַ д
	if(IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	IIC_Send_Byte(reg);           //д�Ĵ�����ַ
	if(IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	for(i = 0;i<len;i++)          //дlen���ֽ�
	{
														    //���һ���ֽ� ��Ӧ��
		IIC_Send_Byte(buf[i]);
		if(IIC_Wait_Ack() == 1)     //�ȴ�Ӧ��
			return 1;
	}
	
	IIC_Stop();                   //��ֹ�ź�
	return 0;
}
