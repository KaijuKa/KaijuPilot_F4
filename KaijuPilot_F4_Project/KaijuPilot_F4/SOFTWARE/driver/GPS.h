#ifndef __GPS_H__
#define __GPS_H__

#include "stm32f4xx.h"

void DRV_GPS_ByteGet(u8 data);
void DRV_GPS_Analysis(u8 len);

#endif
