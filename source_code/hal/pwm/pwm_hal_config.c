#include "pwm_hal_config.h"


#include "at32f4xx_tim.h"



static void PWM1_HALDutyOut(uint32_t duty)
{
	TMR_SetCompare1(TMR3, duty);
}

static void PWM2_HALDutyOut(uint32_t duty)
{
	TMR_SetCompare2(TMR3, duty);
}

static void PWM3_HALDutyOut(uint32_t duty)
{
	TMR_SetCompare3(TMR3, duty);
}

static void PWM4_HALDutyOut(uint32_t duty)
{
	TMR_SetCompare4(TMR3, duty);
}



PWM_HAL_t PWM_HAL[4] = 
{
	{
		.DutyOut = PWM1_HALDutyOut,
	},

	{
		.DutyOut = PWM2_HALDutyOut,
	},

	{
		.DutyOut = PWM3_HALDutyOut,
	},

	{
		.DutyOut = PWM4_HALDutyOut,
	},
};



