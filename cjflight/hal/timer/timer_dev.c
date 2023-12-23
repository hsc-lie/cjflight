#include "timer_dev.h"


static TimerDev_t *TimerDevTable[TIMER_MAX];

static TIMER_DEV_ERROR_t TimerDevCheck(TIMER_t timer)
{
	if(timer >= TIMER_MAX)
	{
		return TIMER_DEV_ERROR_INVALID;
	}

	if(NULL == TimerDevTable[timer])
	{
		return TIMER_DEV_ERROR_NULL;
	}

	return TIMER_DEV_ERROR_OK;
}

TIMER_DEV_ERROR_t TimerDevRegister(TIMER_t timer, TimerDev_t *dev)
{
	if(timer >= TIMER_MAX)
	{
		return TIMER_DEV_ERROR_INVALID;
	}

	TimerDevTable[timer] = dev;

	return TIMER_DEV_ERROR_OK;
}

TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_t timer)
{
	if(timer >= TIMER_MAX)
	{
		return TIMER_DEV_ERROR_INVALID;
	}

	TimerDevTable[timer] = NULL;

	return TIMER_DEV_ERROR_OK;
}


TIMER_DEV_ERROR_t TimerDevInit(TIMER_t timer)
{
	TIMER_DEV_ERROR_t ret;

	ret = TimerDevCheck(timer);
	if(TIMER_DEV_ERROR_OK != ret)
	{
		return ret;
	}

	if(NULL == TimerDevTable[timer]->Init)
	{
		return TIMER_DEV_ERROR_NULL;
	}

	return TimerDevTable[timer]->Init();
}

TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_t timer)
{
	TIMER_DEV_ERROR_t ret;

	ret = TimerDevCheck(timer);
	if(TIMER_DEV_ERROR_OK != ret)
	{
		return ret;
	}

	if(NULL == TimerDevTable[timer]->DeInit)
	{
		return TIMER_DEV_ERROR_NULL;
	}

	return TimerDevTable[timer]->DeInit();
}

TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_t timer, uint8_t channel, uint32_t duty)
{
	TIMER_DEV_ERROR_t ret;

	ret = TimerDevCheck(timer);
	if(TIMER_DEV_ERROR_OK != ret)
	{
		return ret;
	}

	if(NULL == TimerDevTable[timer]->PWMOut)
	{
		return TIMER_DEV_ERROR_NULL;
	}

	return TimerDevTable[timer]->PWMOut(channel, duty);

}