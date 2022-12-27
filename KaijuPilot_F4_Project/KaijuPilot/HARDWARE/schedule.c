#include "schedule.h"
#include "delay.h"
#include "imu.h"
#include "msgInterchange.h"
#include "led.h"
#include "remote_signal.h"
#include "flight_ctrl.h"
#include "pwm.h"

#define TASK_NUM 4


/**
  * @brief :1ms任务
  * @param :无
  * @note  :无
  * @retval:无
  */ 
static void task_1ms(void)
{
	IMU_initial_data_update();
	imu(0.001f);
	calculate_RPY();
	return;
}

/**
  * @brief :5ms任务
  * @param :无
  * @note  :无
  * @retval:无
  */ 
static void task_5ms(void)
{
	return;
}

/**
  * @brief :10ms任务
  * @param :无
  * @note  :无
  * @retval:无
  */ 
static void task_10ms(void)
{
	return;
}

/**
  * @brief :20ms任务
  * @param :无
  * @note  :无
  * @retval:无
  */ 
static void task_20ms(void)
{
	imu_send();
	ch_offline_check(20);
	ch_data_limited();
	
	flight_ctrl_task();
	return;
}

taskstructure task_queue[] = {
	{task_1ms, 0, 1},
	{task_5ms, 0, 5},
	{task_10ms, 0, 10},
	{task_20ms, 0, 20}
};
u32 time_now = 0;

/**
  * @brief :调度器初始化
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void schedule_init(void)
{
	for(u8 i = 0;i < TASK_NUM;i++)
	{
		task_queue[i].last_run = GetSysTime_ms();
	}
}

/**
  * @brief :调度器运行
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void schedule_run(void)
{
	while(1)
	{
		for(u8 i = 0;i < TASK_NUM;i++)
		{
			time_now = GetSysTime_ms();
			if(time_now - task_queue[i].last_run >= task_queue[i].period)
			{
				task_queue[i].task();
				task_queue[i].last_run = time_now;
			}
		}
		led = !led;
	}
}
