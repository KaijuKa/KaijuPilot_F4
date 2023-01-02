#include "task_manage.h"
#include "imu.h"
#include "msg_interchange.h"
#include "led.h"
#include "remote_signal.h"
#include "flight_ctrl.h"

#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
* �� �� ��         : app_task_reate
* ��������		     : RTOS����ǰ�����񴴽�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void app_task_reate(void)
{
	xTaskCreate(task_200ms,           /* �������� */
                "task_200ms",         /* ���������ַ�����ʽ��������� */
                 512,                /* ջ��С����λΪ�֣���4���ֽ� */
                 NULL,               /* �����β� */
                 1,                  /* ���ȼ�����ֵԽ�����ȼ�Խ�� */
                 NULL);  /* ������ */
	
	xTaskCreate(task_1ms,           /* �������� */
                "task_1ms",         /* ���������ַ�����ʽ��������� */
                 2048,                /* ջ��С����λΪ�֣���4���ֽ� */
                 NULL,               /* �����β� */
                 31,                  /* ���ȼ�����ֵԽ�����ȼ�Խ�� */
                 NULL);  /* ������ */
				 
	xTaskCreate(task_20ms,           /* �������� */
                "task_20ms",         /* ���������ַ�����ʽ��������� */
                 1024,                /* ջ��С����λΪ�֣���4���ֽ� */
                 NULL,               /* �����β� */
                 30,                  /* ���ȼ�����ֵԽ�����ȼ�Խ�� */
                 NULL);  /* ������ */
}


/*******************************************************************************
* �� �� ��         : task_1ms
* ��������		     : ִ������Ϊ1ms������ ��Ҫ����̬����
* ��    ��         : ��
* ��    ��         : ��
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
		vTaskDelayUntil( &xLastWakeTime, 1 );
	}
}

/*******************************************************************************
* �� �� ��         : task_20ms
* ��������		     : ִ������Ϊ20ms������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void task_20ms(void *pvParameters)
{
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		RC_Offline_Check(20);
		MSG_Send();
		Flight_Ctrl_Task(20);
		vTaskDelayUntil( &xLastWakeTime, 20 );
	}
}

/*******************************************************************************
* �� �� ��         : task_200ms
* ��������		     : ִ������Ϊ200ms������
* ��    ��         : ��
* ��    ��         : ��
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
