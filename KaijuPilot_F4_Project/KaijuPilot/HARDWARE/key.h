#ifndef __KEY_H__
#define __KEY_H__

#include "io.h"
#include "delay.h"

#define KEY0  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键0

#define KEY0_PRES	1		//KEY0  

void KEY_Init(void);  //按键IO初始化
u8 KEY_Scan(u8 mode);  //按键扫描函数	


#endif
