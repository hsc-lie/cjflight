#ifndef __REMOTE_H_
#define __REMOTE_H_

#include "common.h"




#define REMOTE_VALUE_MIN      1000
#define REMOTE_VALUE_MAX      2000      

#define THROTTLE_TO_MOTOR_MIN      4000
#define THROTTLE_TO_MOTOR_MAX      8000

#define THROTTLE_OUT_MIN           0
#define THROTTLE_OUT_MAX           3000



#define SET_PITCH_MIN             (-45)
#define SET_PITCH_MAX              45

#define SET_ROLL_MIN              (-45)
#define SET_ROLL_MAX               45

#define SET_YAW_RATE_MIN           (-1.0f)
#define SET_YAW_RATE_MAX           (1.0f)



typedef enum
{
    Manual_Mode = 0,
    Stabilize_Mode,
    Auto_Mode,
}flight_mode_t;



void RemoteTask(void * parameters);



#endif /*__REMOTE_H_*/
