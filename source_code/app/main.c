#include "main.h"
#include "at32f4xx.h"

#include "stdio.h"


#include "rcc_config.h"
#include "gpio_config.h"
#include "usart_config.h"
#include "dma_config.h"
#include "timer_config.h"


#include "led.h"


#include "mpu6050_config.h"

#include "simulation_i2c.h"
//#include "bmp280.h"
//#include "spl06.h"

#include "led_cfg.h"
#include "remote_data.h"

#include "remote_task.h"
#include "quaternion.h"
#include "control.h"



/************************************

xPortGetMinimumEverFreeHeapSize();         获取曾经最小堆的剩余大小 单位字节
xPortGetFreeHeapSize();                    获取当前堆的剩余大小 单位字节
uxTaskGetStackHighWaterMark(NULL);         获取当前运行任务启动以来剩余的最小大小  单位4字节

*************************************/


uint32_t heap_size1;
uint32_t heap_size2;

uint32_t stack_size;

/*遥控数据传输队列句柄*/
xQueueHandle RemoteDataQueue;

xQueueHandle printf_velocity_queue;
xQueueHandle printf_position_queue;
xQueueHandle printf_acc_queue;


/*遥控器数据接受信号句柄*/
//xSemaphoreHandle remote_read_semaphore;

void LEDTask(void * parameters)
{
	for(;;)
	{
		LED_SetValue(&LED1, 1);
		vTaskDelay(500);
		LED_SetValue(&LED1, 0);
		vTaskDelay(500);
	}
	
}



int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
  	//uart_send_byte(UARTx, (uint8_t)ch);
	return (ch);
}


void PrintfTask(void * parameters)
{
	float printf_velocity = 0;
	float printf_position = 0;
	float printf_acc = 0;

	for(;;)
	{
		xQueueReceive(printf_velocity_queue,&printf_velocity,0);
		xQueueReceive(printf_position_queue,&printf_position,0);
		xQueueReceive(printf_acc_queue,&printf_acc,0);

		printf("d: %f, %f, %f\n", printf_velocity, printf_position, printf_acc);

		//stack_size = uxTaskGetStackHighWaterMark(NULL); 
		//printf("hello world\n");
		//uart_send_byte(UARTx,6);
		vTaskDelay(20);
	}

}

/*

__asm void _disable_irq()
{

	MOV R0,#1
	MSR PRIMASK,R0

}
__asm void _enable_irq()
{
	cpsid i
}
*/




int main(void)
{
	/*中断优先级分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	RCC_ConfigInitAll();
	GPIO_ConfigInitAll();
	DMA_ConfigInitAll();
	Timer_ConfigInitAll();
	USART_ConfigInitAll();
	

	/*LED初始化*/
	//led_init();

	
	/*PID参数初始化*/
	FilterInit();
	
	/*电机PWM初始化*/
	//pwm_init();

	
	/*气压计初始化*/
	//bmp280_init();
	//spl0601_init();
	
	/*MPU6050初始化*/
	MPU6050_Init(&MPU6050);
	
	//tim_init_ms(20);
	
	/*创建遥控数据消息队列*/
	RemoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));

	printf_velocity_queue = xQueueCreate(1,sizeof(float));
	printf_position_queue = xQueueCreate(1,sizeof(float));
	printf_acc_queue = xQueueCreate(1,sizeof(float));

	/*创建遥控接受信号*/
	//remote_read_semaphore = xSemaphoreCreateBinary();


	/*LED任务创建*/
	xTaskCreate(LEDTask,
				"led",
				32,
				NULL,
				1,
				NULL);

	/*打印任务创建*/
	/*xTaskCreate(PrintfTask,
				"printf",
				200,
				NULL,
				2,
				NULL);*/

	/*遥控器接受任务创建*/
	xTaskCreate(RemoteTask,
				"remote",
				256,
				NULL,
				4,
				NULL);

	/*姿态控制任务创建*/
	xTaskCreate(ControlTask,
				"control",
				512,
				NULL,
				5,
				NULL);


	/*启动调度器*/		
	vTaskStartScheduler();
		
	for(;;)
	{
		
	}

}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif
