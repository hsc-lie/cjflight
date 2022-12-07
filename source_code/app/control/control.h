#ifndef _CONTROL_H_
#define _CONTROL_H_


#include "common.h"


#define ATTITUDE_CONTROL_OUT_MAX     2000

#define MOTOR_STOP_PWM               0

#define MOTOR_PWM_MIN                500
#define MOTOR_PWM_MAX                4000

#define THROTTLE_DEAD_ZONE           100

#define THROTTLE_BASE                500 

#define ALTITUDE_THROTTLE_CONTROL_OUT_MAX     2000

#define PITCH_ZERO                   (-2.00)
#define ROLL_ZERO                    (-1.21)

void FilterInit(void);

void ControlTask(void * parameters);
void Strapdown_INS_High(void);




#endif /*_CONTROL_H_*/
