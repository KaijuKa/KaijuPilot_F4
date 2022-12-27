#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "io.h"

typedef struct{
	void (*task)(void);
	u32 last_run;
	u8 period;
} taskstructure;

static void task_1ms(void);
static void task_5ms(void);
static void task_10ms(void);
static void task_20ms(void);
void schedule_init(void);
void schedule_run(void);

#endif
