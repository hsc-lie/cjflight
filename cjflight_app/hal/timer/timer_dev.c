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


/*
 * @函数名  TimerDevRegister
 * @用  途  定时器设备注册
 * @参  数  timer:注册到的定时器设备号
 *          dev:定时器设备
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevRegister(TIMER_t timer, TimerDev_t *dev)
{
	TIMER_DEV_CHECK(timer);

	TimerDevTable[timer] = dev;

	return TIMER_DEV_ERROR_OK;
}

/*
 * @函数名  TimerDevUnregister
 * @用  途  定时器设备注销
 * @参  数  timer:注销的定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_t timer)
{
	TIMER_DEV_CHECK(timer);

	TimerDevTable[timer] = NULL;

	return TIMER_DEV_ERROR_OK;
}

/*
 * @函数名  TimerDevInit
 * @用  途  定时器设备初始化
 * @参  数  timer:定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevInit(TIMER_t timer)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->Init);

	return TimerDevTable[timer]->Init();
}

/*
 * @函数名  TimerDevDeInit
 * @用  途  定时器设备反初始化
 * @参  数  timer:定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_t timer)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->DeInit);

	return TimerDevTable[timer]->DeInit();
}

/*
 * @函数名  TimerDevGetCount
 * @用  途  获取定时器设备的计数值
 * @参  数  timer:定时器设备号
 *          count:获取到的设备值
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevGetCount(TIMER_t timer, uint32_t *count)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->GetCount);

	return TimerDevTable[timer]->GetCount(count);
}

/*
 * @函数名  TimerDevPWMOut
 * @用  途  定时器设备PWM输出
 * @参  数  timer:定时器设备号
 *          channel:PWM输出的通道号
 *          duty:PWM输出值
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_t timer, uint8_t channel, uint32_t duty)
{
	TIMER_DEV_CHECK_FUNC(timer, TimerDevTable[timer]->PWMOut);

	return TimerDevTable[timer]->PWMOut(channel, duty);
}