#ifndef _CONTROL_H_
#define _CONTROL_H_


#include "common.h"


#define ATTITUDE_CONTROL_OUT_MAX     2000

#define MOTOR_STOP_PWM               4000

#define MOTOR_PWM_MIN                4500
#define MOTOR_PWM_MAX                8000

#define THROTTLE_DEAD_ZONE           100

#define THROTTLE_BASE                500 

#define ALTITUDE_THROTTLE_CONTROL_OUT_MAX     2000

#define PITCH_ZERO                   (-3)
#define ROLL_ZERO                    (-0)

void pid_init(void);

void control_task(void * parameters);
void Strapdown_INS_High(void);




#endif /*_CONTROL_H_*/
