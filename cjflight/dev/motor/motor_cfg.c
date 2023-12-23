#include "motor_cfg.h"


#include "timer_dev.h"

#define MOTOR_OUT_MIN        (0)
#define MOTOR_OUT_MAX        (4000)


static TimerDev_t *MotorPWMDev = NULL; 

static void Motor1Out(int32_t outValue)
{

	if(NULL == MotorPWMDev)
	{
		MotorPWMDev = TimerDevGet(1);
	}

	if(NULL != MotorPWMDev)
	{
		TimerPWMOut(MotorPWMDev, 1, outValue + 4000);
	}
}

static void Motor2Out(int32_t outValue)
{
	if(NULL == MotorPWMDev)
	{
		MotorPWMDev = TimerDevGet(1);
	}

	if(NULL != MotorPWMDev)
	{
		TimerPWMOut(MotorPWMDev, 2, outValue + 4000);
	}
}

static void Motor3Out(int32_t outValue)
{
	if(NULL == MotorPWMDev)
	{
		MotorPWMDev = TimerDevGet(1);
	}

	if(NULL != MotorPWMDev)
	{
		TimerPWMOut(MotorPWMDev, 3, outValue + 4000);
	}
}

static void Motor4Out(int32_t outValue)
{
	if(NULL == MotorPWMDev)
	{
		MotorPWMDev = TimerDevGet(1);
	}

	if(NULL != MotorPWMDev)
	{
		TimerPWMOut(MotorPWMDev, 4, outValue + 4000);
	}
}




Motor_t Motor[4] = 
{
	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor1Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor2Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor3Out,
	},

	{
		.OutMin = MOTOR_OUT_MIN,
		.OutMax = MOTOR_OUT_MAX,
		.Out = Motor4Out,
	},

};

