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
#include "par_manage.h"
#include "pwm.h"
#include "msg_interchange.h"
#include "bmp280.h"
#include "pos_calcu.h"

#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
	//��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//�ж����ȼ��������
	systick_it_init();     //��ʼ���������
	DRV_LED_Init();        //��ʼ��led
	DRV_USART1_Init(57600);//��ʼ������
	RC_Init();             //��ʼ��RC
	DRV_SPI1_Init();       //��ʼ��SPI1
	DRV_SPI2_Init();       //��ʼ��SPI2
	DRV_IIC_Init();        //��ʼ��IIC
	DRV_PWM_Init();        //��ʼ��PWM
	DRV_Icm20602_Init();   //��ʼ��icm20602
	DRV_Bmp280_Init();     //��ʼ��bmp280
	IMU_Init();            //��ʼ��IMU
	POS_Init();            //��ʼ��POS
	DT_MSG_Init();         //��ʼ�����ݴ���
	PAR_Load();            //��ȡȫ������
	
	//����1s��ʾ��ʼ�����
	DRV_LED_LIGHTING(1);
	delay_ms(1000);
	DRV_LED_LIGHTING(0);
	delay_ms(200);

	
	//����RTOS
	RTOS_en = 1;
	
	//��������
	app_task_reate();
	
	//����������
	vTaskStartScheduler();
	while(1)
	{
		;
	}
}

