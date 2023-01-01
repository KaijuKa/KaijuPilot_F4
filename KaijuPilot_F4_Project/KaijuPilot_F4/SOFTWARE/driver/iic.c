#include "iic.h"

/*******************************************************************************
* �� �� ��         : DRV_IIC_Init
* ��������		     : IIC��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_IIC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ�� GPIOC ʱ��
	
	//GPIOC7,C8��ʼ������
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
* �� �� ��         : DRV_DRV_SDA_OUT
* ��������		     : SDA�������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOC8��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
}

/*******************************************************************************
* �� �� ��         : DRV_DRV_SDA_IN
* ��������		     : SDA��������	   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	//GPIOC8��ʼ������
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��
}

/*******************************************************************************
* �� �� ��         : DRV_DRV_IIC_Start
* ��������		     : ����IIC��ʼ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_IIC_Start(void)
{
	DRV_SDA_OUT();     //sda�����
	IIC_SDA_HIGH;	  	  
	IIC_SCL_HIGH;
	delay_us(5);
 	IIC_SDA_LOW;//START:when CLK is high,DATA change form high to low 
	delay_us(6);
	IIC_SCL_LOW;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/*******************************************************************************
* �� �� ��         : DRV_IIC_Stop
* ��������		     : ����IICֹͣ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void DRV_IIC_Stop(void)
{
	DRV_SDA_OUT();//sda�����
	IIC_SCL_LOW;
	IIC_SDA_LOW;//STOP:when CLK is high DATA change form low to high
 	IIC_SCL_HIGH; 
	delay_us(6); 
	IIC_SDA_HIGH;//����I2C���߽����ź�
	delay_us(6);							   	
}

/*******************************************************************************
* �� �� ��         : DRV_IIC_Wait_Ack
* ��������		     : �ȴ�Ӧ���źŵ���   
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
u8 DRV_IIC_Wait_Ack(void)
{
	u8 tempTime=0;
	DRV_SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL_LOW;//ʱ�����0 	   
	return 0;  
} 

/*******************************************************************************
* �� �� ��         : DRV_IIC_Ack
* ��������		     : ����ACKӦ��  
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : DRV_IIC_NAck
* ��������		     : ����NACK��Ӧ��  
* ��    ��         : ��
* ��    ��         : ��
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
* �� �� ��         : DRV_IIC_Send_Byte
* ��������		     : IIC����һ���ֽ� 
* ��    ��         : txd������һ���ֽ�
* ��    ��         : 0���ͳɹ� 1����ʧ��
*******************************************************************************/		  
u8 DRV_IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	DRV_SDA_OUT(); 	    
	IIC_SCL_LOW;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
		if((txd&0x80)>0) //0x80  1000 0000
			IIC_SDA_HIGH;
		else
			IIC_SDA_LOW;
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_HIGH;
		delay_us(2); 
		IIC_SCL_LOW;	
		delay_us(2);
	}	
	return DRV_IIC_Wait_Ack();
} 

/*******************************************************************************
* �� �� ��         : DRV_IIC_Read_Byte
* ��������		     : IIC��һ���ֽ� 
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK 
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/  
u8 DRV_IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	DRV_SDA_IN();//SDA����Ϊ����
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
		DRV_IIC_NAck();//����nACK
	else
		DRV_IIC_Ack(); //����ACK   
	return receive;
}

/*******************************************************************************
* �� �� ��         : DRV_IIC_Read_Str
* ��������		     : IIC������ֽ�
* ��    ��         : �豸��ַ���Ĵ�����ַ����ȡ���ȣ���ȡ����
* ��    ��         : 1 ʧ�� 0 �ɹ�
*******************************************************************************/
u8 DRV_IIC_Read_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	DRV_IIC_Start();                  //��ʼ�ź�
	DRV_IIC_Send_Byte((addr<<1) | 0); //д�豸��ַ д
	if(DRV_IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	DRV_IIC_Send_Byte(reg);           //д�Ĵ�����ַ
	if(DRV_IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	DRV_IIC_Start();                  //��ʼ�ź�
	DRV_IIC_Send_Byte((addr<<1) | 1); //д�豸��ַ ��
	if(DRV_IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	for(i = 0;i<len;i++)          //��ȡlen���ֽ�
	{
		if(i == len-1)              //���һ���ֽ� ��Ӧ��
			buf[i] = DRV_IIC_Read_Byte(0);
		else               
			buf[i] = DRV_IIC_Read_Byte(1);
	}
	
	DRV_IIC_Stop();                   //��ֹ�ź�
	return 0;
}

/*******************************************************************************
* �� �� ��         : DRV_IIC_Write_Str
* ��������		     : IICд����ֽ�
* ��    ��         : �豸��ַ���Ĵ�����ַ����ȡ���ȣ���ȡ����
* ��    ��         : 1 ʧ�� 0 �ɹ�
*******************************************************************************/
u8 DRV_IIC_Write_Str(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i;
	
	DRV_IIC_Start();                  //��ʼ�ź�
	DRV_IIC_Send_Byte((addr<<1) | 0); //д�豸��ַ д
	if(DRV_IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	DRV_IIC_Send_Byte(reg);           //д�Ĵ�����ַ
	if(DRV_IIC_Wait_Ack() == 1)       //�ȴ�Ӧ��
		return 1;
	
	for(i = 0;i<len;i++)          //дlen���ֽ�
	{
														    //���һ���ֽ� ��Ӧ��
		DRV_IIC_Send_Byte(buf[i]);
		if(DRV_IIC_Wait_Ack() == 1)     //�ȴ�Ӧ��
			return 1;
	}
	
	DRV_IIC_Stop();                   //��ֹ�ź�
	return 0;
}
