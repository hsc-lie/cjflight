#include "motor_cfg.h"


#include "pwm_hal_cfg.h"

#define MOTOR_OUT_MIN        (0)
#define MOTOR_OUT_MAX        (4000)



static void Motor1_Out(int32_t outValue)
{
	PWM_HALDutyOut(&PWM_HAL[0], outValue + 4000);
}

static void Motor2_Out(int32_t outValue)
{
	PWM_HALDutyOut(&PWM_HAL[1], outValue + 4000);
}

static void Motor3_Out(int32_t outValue)
{
	PWM_HALDutyOut(&PWM_HAL[2], outValue + 4000);
}

static void Motor4_Out(int32_t outValue)
{
	PWM_HALDutyOut(&PWM_HAL[3], outValue + 4000);
}




Motor_t Motor[4] = 
{
	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor1_Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor2_Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor3_Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor4_Out,
	},

};

