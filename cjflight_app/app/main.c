#include "main.h"

#include "stdio.h"

#include "gpio_dev.h"
#include "usart_dev.h"
#include "i2c_dev.h"
#include "timer_dev.h"

#include "mpu6050_cfg.h"
#include "bmp280_cfg.h"
#include "spl06_cfg.h"

#include "led_cfg.h"
#include "remote_data.h"

#include "sensors.h"
#include "remote.h"
#include "control.h"



/************************************

xPortGetMinimumEverFreeHeapSize();         获取曾经最小堆的剩余大小 单位字节
xPortGetFreeHeapSize();                    获取当前堆的剩余大小 单位字节
uxTaskGetStackHighWaterMark(NULL);         获取当前运行任务启动以来剩余的最小大小  单位4字节

*************************************/


uint32_t heap_size1;
uint32_t heap_size2;
uint32_t stack_size;


void LEDTask(void * parameters)
{
	for(;;)
	{
		LEDSetValue(&LED[0], 1);
		LEDSetValue(&LED[1], 1);
		LEDSetValue(&LED[2], 1);
		vTaskDelay(500);
		LEDSetValue(&LED[0], 0);
		LEDSetValue(&LED[1], 0);
		LEDSetValue(&LED[2], 0);
		vTaskDelay(500);
	}
	
}


int _write (int fd, char *pBuffer, int size)
{
	USARTDevSendData(USART_DEV_PRINTF, (uint8_t *)pBuffer, (uint32_t)size);

	return size;
}


void PrintTask(void * parameters)
{

	//float baroAltitude = 0;
	//float altitude = 0;
	//float acc = 0;

	for(;;)
	{
		
		//printf("d: %f, %f, %f\n", printf_velocity, printf_position, printf_acc);
		//printf("d: %f, %f, %f\n", baroAltitude, altitude, acc);
		//printf("d: %f, %f, %f\n", 0.6, 6.6, 6.66);
		//printf("hello world\n");
		//USART_HAL_SendData(&USART1_HAL, "hello world\n", 12);

		//printf("hello world\n");
		//stack_size = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelay(10);	
	}

}


#if 0
void I2C_DevAddrTest()
{
	uint8_t addr = 0;

	uint8_t reg = 0;
	uint8_t readData = 0;

	I2C_DEV_ERROR_t ret;

	for(addr = 0;addr < 0x7f;addr++)
	{
		//ret = SimulationI2CReadData(&SimulationI2C1, addr, &reg, 1, &readData, 1);
		//ret = I2C_HalReadData(&I2C_Dev1, addr, &reg, 1, &readData, 1);

		if(I2C_DEV_ERROR_OK == ret)
		{
			//ret = 0;
		}
	}
}
#endif


int main(void)
{
	GPIODevInit();
	I2CDevInit(I2C_DEV_SENSOR);
	USARTDevInit(USART_DEV_PRINTF);
	USARTDevInit(USART_DEV_REMOTE);
	TimerDevInit(TIMER_DEV_TEST);
	TimerDevInit(TIMER_DEV_MOTOR_PWM);

	/*MPU6050初始化*/
	MPU6050Init(&MPU6050);

	/*气压计初始化*/
	//BMP280_Init(&BMP280);
	//SPL06Init(&SPL06);
	//I2C_DevAddrTest();
	
	//创建遥控数据消息队列
	RemoteDataToControlQueue = xQueueCreate(1, sizeof(RemoteData_t));
	RemoteDataToPrintQueue = xQueueCreate(1, sizeof(RemoteData_t));

	SensorsDataMutex = xSemaphoreCreateMutex();

	/*LED任务创建*/
	xTaskCreate(LEDTask,
				"led",
				32,
				NULL,
				2,
				NULL);

	/*打印任务创建*/
	xTaskCreate(PrintTask,
				"print",
				256,
				NULL,
				1,
				NULL);
	
	/*传感器采集任务创建*/
	xTaskCreate(SensorsTask,
				"sensors",
				256,
				NULL,
				4,
				NULL);

	/*遥控器接受任务创建*/
	xTaskCreate(RemoteTask,
				"remote",
				256,
				NULL,
				3,
				NULL);

	/*姿态控制任务创建*/
	xTaskCreate(ControlTask,
				"control",
				256,
				NULL,
				5,
				NULL);

	/*启动调度器*/		
	vTaskStartScheduler();

	for(;;)
	{
		
	}
}
