#ifndef _REMOTE_H_
#define _REMOTE_H_

#include "common.h"

#include "sbus.h"
#include "tim_input_capture.h"
#include "pwm.h"


#define REMOTE_VALUE_MIN      1000
#define REMOTE_VALUE_MAX      2000      

#define THROTTLE_TO_MOTOR_MIN      4000
#define THROTTLE_TO_MOTOR_MAX      8000

#define THROTTLE_OUT_MIN           0
#define THROTTLE_OUT_MAX           3000



#define SET_PITCH_MIN             (-35)
#define SET_PITCH_MAX              35

#define SET_ROLL_MIN              (-35)
#define SET_ROLL_MAX               35

#define SET_YAW_RATE_MIN           (-1.0f)
#define SET_YAW_RATE_MAX           (1.0f)



typedef enum
{
    Manual_Mode = 0,
    Stabilize_Mode,
    Auto_Mode,
}flight_mode_t;

typedef struct
{
    flight_mode_t mode; 
    uint32_t throttle_out;
    float set_pitch;
    float set_roll;
    float set_yaw_rate;
}remote_data_t; 




void remote_init(void);
uint8_t remote_update(void);
uint16_t remote_channel_value_get(IBUS_Channel_Type channel_name);
int remote_throttle_to_motor_get(void);

void remote_task(void * parameters);



#endif /*_REMOTE_H_*/
