#ifndef __AT24C02_H__
#define __AT24C02_H__

#include "io.h"

void AT24_Read_Str(u8* str, u8 len, u8 addr);
void AT24_Write_Str(u8* str, u8 len, u8 addr);

#endif
