#ifndef __PWM_HAL_H_
#define __PWM_HAL_H_


#include "common.h"


typedef struct
{
	void (* DutyOut)(uint32_t duty);
}PWM_HAL_t;


void PWM_HALDutyOut(PWM_HAL_t * pwm, uint32_t duty);



#endif /*__PWM_HAL_H_*/
