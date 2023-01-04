#include "task_manage.h"
#include "imu.h"
#include "msg_interchange.h"
#include "led.h"
#include "remote_signal.h"
#include "flight_ctrl.h"
#include "par_manage.h"

#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
* 函 数 名         : app_task_reate
* 函数功能		     : RTOS启动前的任务创建
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void app_task_reate(void)
{
	xTaskCreate(task_200ms,           /* 任务函数名 */
                "task_200ms",         /* 任务名，字符串形式，方便调试 */
                 512,                /* 栈大小，单位为字，即4个字节 */
                 NULL,               /* 任务形参 */
                 1,                  /* 优先级，数值越大，优先级越高 */
                 NULL);  /* 任务句柄 */
	
	xTaskCreate(task_1ms,           /* 任务函数名 */
                "task_1ms",         /* 任务名，字符串形式，方便调试 */
                 2048,                /* 栈大小，单位为字，即4个字节 */
                 NULL,               /* 任务形参 */
                 31,                  /* 优先级，数值越大，优先级越高 */
                 NULL);  /* 任务句柄 */
				 
	xTaskCreate(task_20ms,           /* 任务函数名 */
                "task_20ms",         /* 任务名，字符串形式，方便调试 */
                 2048,                /* 栈大小，单位为字，即4个字节 */
                 NULL,               /* 任务形参 */
                 30,                  /* 优先级，数值越大，优先级越高 */
                 NULL);  /* 任务句柄 */
}


/*******************************************************************************
* 函 数 名         : task_1ms
* 函数功能		     : 执行周期为1ms的任务 主要放姿态解算
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void task_1ms(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		IMU_Data_Update();
		IMU_Calcu(0.001);
		IMU_RPY_Calcu();
		IMU_Data_Share();
		MSG_Ctrl_Task(1);
		PAR_Store_Task(1);
		vTaskDelayUntil( &xLastWakeTime, 1 );
	}
}

/*******************************************************************************
* 函 数 名         : task_20ms
* 函数功能		     : 执行周期为20ms的任务
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void task_20ms(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		RC_Offline_Check(20);
		RC_Data_Share();
		Flight_Ctrl_Task(20);
		Flight_Data_Share();
		vTaskDelayUntil( &xLastWakeTime, 20 );
	}
}

/*******************************************************************************
* 函 数 名         : task_200ms
* 函数功能		     : 执行周期为200ms的任务
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void task_200ms(void *pvParameters)
{
	while(1)
	{
		DRV_LED_LIGHTING(1);
		vTaskDelay( 200 );
		DRV_LED_LIGHTING(0);
		vTaskDelay( 200 );
	}
}
