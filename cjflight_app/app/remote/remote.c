#include "remote.h"

#include "remote_data.h"

#include "ibus.h"

#include "usart_dev.h"
#include "timer_dev.h"


//遥控器数据到控制模块队列
xQueueHandle RemoteDataToControlQueue;
//遥控器数据到打印模块队列
xQueueHandle RemoteDataToPrintQueue;

static IBus_t IBus;

static void IBusDataToRemoteData(IBus_t * ibus, RemoteData_t * remote)
{
	int throttle = 0;
	int16_t ibusChannelValue;
	int16_t remoteValue;

	/**********************油门**************************/
    throttle = IBusGetChannelData(ibus, IBUS_Channel3);
    throttle = IntRange(throttle, REMOTE_VALUE_MIN, REMOTE_VALUE_MAX);

    throttle = (THROTTLE_TO_MOTOR_MAX - THROTTLE_TO_MOTOR_MIN) * (throttle - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
	throttle += THROTTLE_TO_MOTOR_MIN;

	ibusChannelValue = IBusGetChannelData(ibus, IBUS_Channel3); 

	remoteValue = (THROTTLE_OUT_MAX - THROTTLE_OUT_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + THROTTLE_OUT_MIN;
	RemoteDataSetRockerValue(remote, REMOTE_DATA_LEFT_ROCKER_Y, remoteValue);
	
	ibusChannelValue = IBusGetChannelData(ibus, IBUS_Channel2);
	remoteValue = (SET_PITCH_MAX - SET_PITCH_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_PITCH_MIN;
	RemoteDataSetRockerValue(remote, REMOTE_DATA_RIGHT_ROCKER_Y, remoteValue);

	ibusChannelValue = IBusGetChannelData(ibus, IBUS_Channel1);
	remoteValue = (SET_ROLL_MAX - SET_ROLL_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_ROLL_MIN;
	RemoteDataSetRockerValue(remote, REMOTE_DATA_RIGHT_ROCKER_X, remoteValue);

	ibusChannelValue = IBusGetChannelData(ibus, IBUS_Channel4);
	remoteValue = (SET_YAW_RATE_MAX - SET_YAW_RATE_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_YAW_RATE_MIN;
	RemoteDataSetRockerValue(remote, REMOTE_DATA_LEFT_ROCKER_X, remoteValue);

	ibusChannelValue = IBusGetChannelData(ibus, IBUS_Channel7);
	if((ibusChannelValue > 750) && (ibusChannelValue < 1250))
	{
		remoteValue = Stabilize_Mode;
	}
	else if((ibusChannelValue >= 1250) && (ibusChannelValue < 1750))
	{
		remoteValue = Stabilize_Mode;
	}
	else if((ibusChannelValue >= 1750) && (ibusChannelValue < 2250))
	{
		remoteValue = Auto_Mode;
	}
	RemoteDataSetSWValue(remote, REMOTE_DATA_SW4, remoteValue);
    
}

static void IBusDataUpdate()
{
	uint8_t data[128];
	uint32_t outLen = 0;
	RemoteData_t remoteData = {0};

	uint32_t startTimerCount;
	uint32_t endTimerCount;
	uint32_t timerDiff;

	TimerDevGetCount(TIMER_TEST, &startTimerCount);
	
	USARTDevReadData(USART_REMOTE, data, sizeof(data), &outLen);
	
	if(TRUE == IBusAnalysisData(&IBus, data, outLen))
	{
		IBusDataToRemoteData(&IBus, &remoteData);
		xQueueSend(RemoteDataToControlQueue,&remoteData,0);
		xQueueSend(RemoteDataToPrintQueue,&remoteData,0);
	}

	TimerDevGetCount(TIMER_TEST, &endTimerCount);
	timerDiff = endTimerCount - startTimerCount;
	(void)timerDiff;

}

void RemoteTask(void * parameters)
{
	for(;;)
	{
        //xSemaphoreTake(remote_read_semaphore,portMAX_DELAY);
        IBusDataUpdate();
		vTaskDelay(5);
	}
}
