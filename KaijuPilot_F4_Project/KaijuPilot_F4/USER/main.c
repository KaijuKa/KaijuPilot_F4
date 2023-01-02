#include "stm32f4xx.h"
#include "uart.h"
#include "delay.h"
#include "spi.h"
#include "icm20602.h"
#include "led.h"
#include "imu.h"
#include "task_manage.h"
#include "remote_signal.h"
#include "iic.h"
#include "arg_manage.h"
#include "pwm.h"

#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
	//初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//中断优先级组别设置
	systick_it_init();     //初始化裸机调度
	DRV_LED_Init();        //初始化led
	DRV_USART1_Init(57600);//初始化数传
	RC_Init();             //初始化RC
	DRV_SPI_Init();        //初始化SPI
	DRV_IIC_Init();        //初始化IIC
	DRV_PWM_Init();        //初始化PWM
	DRV_Icm20602_Init();   //初始化icm20602
	IMU_Init();            //初始化IMU
//	ARG_Load();            //读取全部参数
	
	//常亮1s表示初始化完成
	DRV_LED_LIGHTING(1);
	delay_ms(1000);
	DRV_LED_LIGHTING(0);
	delay_ms(200);

	//启动RTOS
	RTOS_en = 1;
	
	//创建任务
	app_task_reate();
	
	//启动调度器
	vTaskStartScheduler();
	while(1)
	{
		;
	}
}

