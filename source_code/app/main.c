#include "main.h"
#include "at32f4xx.h"

#include "stdio.h"


#include "rcc_config.h"
#include "gpio_config.h"
#include "usart_config.h"
#include "dma_config.h"
#include "timer_config.h"



#include "usart_hal_config.h"
#include "i2c_hal_config.h"
#include "simulation_i2c_config.h"



#include "mpu6050_config.h"
#include "bmp280_cfg.h"
#include "spl06_cfg.h"


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


xQueueHandle BaroAltitudeQueue;
xQueueHandle AltitudeQueue;
xQueueHandle AccQueue;





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

	uint8_t data = (uint8_t)ch;
  	USART_HAL_SendData(&USART1_HAL, &data, 1);
	return (ch);
}


void PrintfTask(void * parameters)
{

	float baroAltitude = 0;
	float altitude = 0;
	float acc = 0;
	
	for(;;)
	{
		xQueueReceive(BaroAltitudeQueue,&baroAltitude,0);
		xQueueReceive(AltitudeQueue,&altitude,0);
		xQueueReceive(AccQueue,&acc,0);
		

		//printf("d: %f, %f, %f\n", printf_velocity, printf_position, printf_acc);
		printf("d: %f, %f, %f\n", baroAltitude, altitude, acc);
		
		//USART_HAL_SendData(&USART1_HAL, "hello world\n", 12);

		//printf("hello world\n");


		//stack_size = uxTaskGetStackHighWaterMark(NULL);
		//vTaskDelay(10);
		 
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


void I2C_DevAddrTest()
{
	uint8_t addr = 0;

	uint8_t reg = 0;
	uint8_t readData = 0;

	E_I2C_ERROR ret;

	for(addr = 0;addr < 0x7f;addr++)
	{
		ret = SimulationI2C_ReadData(&SimulationI2C1, addr, &reg, 1, &readData, 1);
		//ret = I2C_HalReadData(&I2C_Dev1, addr, &reg, 1, &readData, 1);

		if(E_I2C_ERROR_OK == ret)
		{
			ret = 0;
		}
	}
}



int main(void)
{
	/*中断优先级分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	RCC_ConfigInitAll();
	GPIO_ConfigInitAll();
	DMA_ConfigInitAll();
	Timer_ConfigInitAll();
	USART_ConfigInitAll();
	

	/*滤波参数初始化*/
	FilterInit();

	/*MPU6050初始化*/
	//MPU6050_Init(&MPU6050);


	/*气压计初始化*/
	//BMP280_Init(&BMP280);
	SPL06_Init(&SPL06);
	//I2C_DevAddrTest();
	

	
	/*创建遥控数据消息队列*/
	RemoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));

	BaroAltitudeQueue = xQueueCreate(1,sizeof(float));
	AltitudeQueue = xQueueCreate(1,sizeof(float));
	AccQueue = xQueueCreate(1,sizeof(float));

	/*创建遥控接受信号*/
	//remote_read_semaphore = xSemaphoreCreateBinary();


	/*LED任务创建*/
	xTaskCreate(LEDTask,
				"led",
				32,
				NULL,
				2,
				NULL);

	/*打印任务创建*/
	xTaskCreate(PrintfTask,
				"printf",
				256,
				NULL,
				1,
				NULL);

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
