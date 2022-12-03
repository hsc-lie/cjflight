#include "pwm_hal.h"




void PWM_HALDutyOut(PWM_HAL_t * pwm, uint32_t duty)
{
	if(NULL != pwm)
	{
		pwm->DutyOut(duty);
	}
}



