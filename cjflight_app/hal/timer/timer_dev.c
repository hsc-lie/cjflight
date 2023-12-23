#include "timer_dev.h"



#define TIMER_DEV_CHECK(timer) \
if((timer) >= TIMER_MAX)\
{\
	return TIMER_DEV_ERROR_INVALID;\
}\
while(0)

#define TIMER_DEV_CHECK_FUNC(timer, func) \
TIMER_DEV_CHECK(timer);\
if((NULL == TimerDevTable[timer]) || (NULL == (func)))\
{\
	return TIMER_DEV_ERROR_NULL;\
}\
while(0)


static TimerDev_t *TimerDevTable[TIMER_MAX];

TIMER_DEV_ERROR_t TimerDevRegister(TIMER_t timer, TimerDev_t *dev)
{
	TIMER_DEV_CHECK(timer);

	TimerDevTable[timer] = dev;

	return TIMER_DEV_ERROR_OK;
}

TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_t timer)
{
	TIMER_DEV_CHECK(timer);

	TimerDevTable[timer] = NULL;

	return TIMER_DEV_ERROR_OK;
}


TIMER_DEV_ERROR_t TimerDevInit(TIMER_t timer)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->Init);

	return TimerDevTable[timer]->Init();
}

TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_t timer)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->DeInit);

	return TimerDevTable[timer]->DeInit();
}

TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_t timer, uint8_t channel, uint32_t duty)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->PWMOut);

	return TimerDevTable[timer]->PWMOut(channel, duty);
}