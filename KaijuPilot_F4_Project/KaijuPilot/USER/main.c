//////////////////////////////////////////////////////////////////////////////////	 
/**************************************************************************************************/	
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/		
#include "delay.h"
#include "uart.h"
#include "led.h"
#include "spi.h"
#include "icm20602.h"
#include "imu.h"
#include "schedule.h"
#include "remote_signal.h"
#include "pwm.h"

int main(void)
{	
	
	SystemInit();//初始化RCC 设置系统主频为72MHZ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_IT_Init();//初始化系统时钟中断
	drv_spi_init();
	TIM1_CH1_Input_Init(0xffff-1, 72-1);
	TIM4_PWM_Init(20000-1, 72-1);
	uart1_init(57600);
	LED_Init();
	Drv_Icm20602_Init();
	imu_Init();
	schedule_init();
	schedule_run();
}

