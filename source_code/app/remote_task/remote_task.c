#include "remote_task.h"

#include "main.h"

#include "usart_hal_config.h"
#include "ibus.h"

#include "remote_data.h"


IBUS_t IBUS = 
{
	0
};




void remote_init()
{
    //tim_input_capture_init();
}


static void IBUSDataToRemoteData(IBUS_t * ibus, RemoteData_t * remote)
{
	int throttle = 0;

	/**********************油门**************************/
    throttle = IBUS_GetChannelData(ibus, E_IBUS_Channel3);
    throttle = int_range(throttle, REMOTE_VALUE_MIN, REMOTE_VALUE_MAX);

    throttle = (THROTTLE_TO_MOTOR_MAX - THROTTLE_TO_MOTOR_MIN) * (throttle - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
	throttle += THROTTLE_TO_MOTOR_MIN;


	
	int16_t ibusChannelValue;
	int16_t remoteValue;
	
	ibusChannelValue = IBUS_GetChannelData(ibus, E_IBUS_Channel3); 

	remoteValue = (THROTTLE_OUT_MAX - THROTTLE_OUT_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + THROTTLE_OUT_MIN;
	RemoteData_SetRockerValue(remote, E_REMOTE_DATA_LEFT_ROCKER_Y, remoteValue);
	

	ibusChannelValue = IBUS_GetChannelData(ibus, E_IBUS_Channel2);
	remoteValue = (SET_PITCH_MAX - SET_PITCH_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_PITCH_MIN;
	RemoteData_SetRockerValue(remote, E_REMOTE_DATA_RIGHT_ROCKER_Y, remoteValue);

	ibusChannelValue = IBUS_GetChannelData(ibus, E_IBUS_Channel1);
	remoteValue = (SET_ROLL_MAX - SET_ROLL_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_ROLL_MIN;
	RemoteData_SetRockerValue(remote, E_REMOTE_DATA_RIGHT_ROCKER_X, remoteValue);

	ibusChannelValue = IBUS_GetChannelData(ibus, E_IBUS_Channel4);
	remoteValue = (SET_YAW_RATE_MAX - SET_YAW_RATE_MIN) * (ibusChannelValue - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN) + SET_YAW_RATE_MIN;
	RemoteData_SetRockerValue(remote, E_REMOTE_DATA_LEFT_ROCKER_X, remoteValue);

	ibusChannelValue = IBUS_GetChannelData(ibus, E_IBUS_Channel7);
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
	RemoteData_SetSWValue(remote, E_REMOTE_DATA_SW4, remoteValue);
    
}



static void IBUSDataUpdate()
{
	uint8_t isGetIBUSPackage;
	uint8_t data[32];
	uint32_t outLen = 0;

	RemoteData_t remoteData = {0};
	
	USART_HAL_ReadData(&USART2_HAL, data, 32, &outLen);

	IBUS_AnalysisData(&IBUS, data, outLen, &isGetIBUSPackage);

	
	if(TRUE == isGetIBUSPackage)
	{
		IBUSDataToRemoteData(&IBUS, &remoteData);

		xQueueSend(RemoteDataQueue,&remoteData,0);
	}


}


void remote_task(void * parameters)
{
    extern xQueueHandle remote_queue;
    extern xSemaphoreHandle remote_read_semaphore;

    //uint8_t remote_update_status;
    //remote_data_t remote_data;
    

	//uint32_t motor_out;
	for(;;)
	{
        //xSemaphoreTake(remote_read_semaphore,portMAX_DELAY);

		
        IBUSDataUpdate();
		vTaskDelay(1);
		/*motor_out = remote_throttle_to_motor_get();
		pwm_out(PWM_Channel_1, motor_out);
		pwm_out(PWM_Channel_2, motor_out);
		pwm_out(PWM_Channel_3, motor_out);
		pwm_out(PWM_Channel_4, motor_out);*/
		//ibus_data_solution();
		//stack_size = uxTaskGetStackHighWaterMark(NULL);
	}
}
