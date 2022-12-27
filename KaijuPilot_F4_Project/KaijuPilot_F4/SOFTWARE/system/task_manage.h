#ifndef __TASK_MANAGE_H__
#define __TASK_MANAGE_H__

#include "stm32f4xx.h"

void app_task_reate(void);
void task_1ms(void *pvParameters);
void task_20ms(void *pvParameters);
void task_200ms(void *pvParameters);
#endif
