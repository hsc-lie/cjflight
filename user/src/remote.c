#include "remote.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"



void remote_init()
{
    //tim_input_capture_init();
    sbus_init();
}


uint8_t remote_update()
{
    //tim_get_ppm();
    uint8_t update_status;

    update_status = ibus_data_solution();

    return update_status;
}


/*int remote_channel_value_get(PPM_Channel_Type channel_name)
{
    return (int)get_ppm_channel_value(channel_name);
}*/

uint16_t remote_channel_value_get(IBUS_Channel_Type channel_name)
{
    return (uint16_t)ibus_get_channel_value(channel_name);
}


int remote_throttle_to_motor_get()
{
    int throttle_out = 0;
    int remote_throttle_value = 0;
    
    remote_throttle_value = remote_channel_value_get(IBUS_Channel3);
    remote_throttle_value = int_range(remote_throttle_value, REMOTE_VALUE_MIN, REMOTE_VALUE_MAX);

    throttle_out = (THROTTLE_TO_MOTOR_MAX - THROTTLE_TO_MOTOR_MIN) * (remote_throttle_value - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
	throttle_out += THROTTLE_TO_MOTOR_MIN;
	
    return throttle_out;
}

void remote_ibus_to_reality(remote_data_t * data)
{
    uint16_t ibus_value;

    ibus_value = ibus_get_channel_value(IBUS_Channel3);   
    data->throttle_out = (THROTTLE_OUT_MAX - THROTTLE_OUT_MIN) * (ibus_value - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
    data->throttle_out += THROTTLE_OUT_MIN;

    ibus_value = ibus_get_channel_value(IBUS_Channel2);
    data->set_pitch = (SET_PITCH_MAX - SET_PITCH_MIN) * (ibus_value - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
    data->set_pitch += SET_PITCH_MIN;

    ibus_value = ibus_get_channel_value(IBUS_Channel1);
    data->set_roll = (SET_ROLL_MAX - SET_ROLL_MIN) * (ibus_value - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
    data->set_roll += SET_ROLL_MIN;

    ibus_value = ibus_get_channel_value(IBUS_Channel4);
    data->set_yaw_rate = (SET_YAW_RATE_MAX - SET_YAW_RATE_MIN) * (ibus_value - REMOTE_VALUE_MIN)/(REMOTE_VALUE_MAX - REMOTE_VALUE_MIN);
    data->set_yaw_rate += SET_YAW_RATE_MIN;
	data->set_yaw_rate = -data->set_yaw_rate;

    ibus_value = ibus_get_channel_value(IBUS_Channel7);
    if((ibus_value > 750) && (ibus_value < 1250))
    {
        data->mode = Stabilize_Mode;
    }
    else if((ibus_value >= 1250) && (ibus_value < 1750))
    {
        data->mode = Stabilize_Mode;
    }
    else if((ibus_value >= 1750) && (ibus_value < 2250))
    {
        data->mode = Auto_Mode;
    }

}


void remote_task(void * parameters)
{
    extern xQueueHandle remote_queue;
    extern xSemaphoreHandle remote_read_semaphore;

    uint8_t remote_update_status;
    remote_data_t remote_data;
    

	//uint32_t motor_out;
	for(;;)
	{
        xSemaphoreTake(remote_read_semaphore,portMAX_DELAY);

		remote_update_status = remote_update();

        if(0 == remote_update_status)
        {
            remote_ibus_to_reality(&remote_data);
            xQueueSend(remote_queue,&remote_data,0);
        }

		/*motor_out = remote_throttle_to_motor_get();
		pwm_out(PWM_Channel_1, motor_out);
		pwm_out(PWM_Channel_2, motor_out);
		pwm_out(PWM_Channel_3, motor_out);
		pwm_out(PWM_Channel_4, motor_out);*/
		//ibus_data_solution();
		//stack_size = uxTaskGetStackHighWaterMark(NULL);
	}
}
