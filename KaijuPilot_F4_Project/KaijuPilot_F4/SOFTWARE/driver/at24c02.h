#ifndef __AT24C02_H__
#define __AT24C02_H__

#include "stm32f4xx.h"

void DRV_AT24_Read_Str(u8* str, u8 len, u8 addr);
void DRV_AT24_Write_Str(u8* str, u8 len, u8 addr);
void DRV_AT24_Write_Byte(u8 byte, u8 addr);

#endif
