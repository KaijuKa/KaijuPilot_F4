#ifndef __KEY_H__
#define __KEY_H__

#include "io.h"
#include "delay.h"

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//��ȡ����0

#define KEY0_PRES	1		//KEY0  

void KEY_Init(void);  //����IO��ʼ��
u8 KEY_Scan(u8 mode);  //����ɨ�躯��	


#endif
