#include "main.h"
#include "at32f4xx.h"

#include "stdio.h"

#include "rcc_cfg.h"
#include "gpio_cfg.h"
#include "usart_cfg.h"
#include "dma_cfg.h"
#include "timer_cfg.h"

#include "usart_hal_cfg.h"
#include "i2c_hal_cfg.h"
#include "simulation_i2c_cfg.h"

#include "mpu6050_cfg.h"
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
		LED_SetValue(&LED2, 1);
		LED_SetValue(&LED3, 1);
		vTaskDelay(500);
		LED_SetValue(&LED1, 0);
		LED_SetValue(&LED2, 0);
		LED_SetValue(&LED3, 0);
		vTaskDelay(500);
	}
	
}



int _write (int fd, char *pBuffer, int size)
{

  	USART_HAL_SendData(&USART1_HAL, (uint8_t *)pBuffer, (uint32_t)size);
	
	return size;
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
		//printf("d: %f, %f, %f\n", baroAltitude, altitude, acc);
		//printf("d: %f, %f, %f\n", 0.6, 6.6, 6.66);
		printf("hello world\n");
		//USART_HAL_SendData(&USART1_HAL, "hello world\n", 12);

		//printf("hello world\n");


		//stack_size = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelay(20);
		 
	}

}



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
	
	//LED_SetValue(&LED1, 0);
	//LED_SetValue(&LED2, 0);
	//LED_SetValue(&LED3, 0);

	/*滤波参数初始化*/
	FilterInit();

	/*MPU6050初始化*/
	MPU6050_Init(&MPU6050);


	/*气压计初始化*/
	//BMP280_Init(&BMP280);
	//SPL06_Init(&SPL06);
	//I2C_DevAddrTest();
	

	
	/*创建遥控数据消息队列*/
	RemoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));

	BaroAltitudeQueue = xQueueCreate(1,sizeof(float));
	AltitudeQueue = xQueueCreate(1,sizeof(float));
	AccQueue = xQueueCreate(1,sizeof(float));


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
