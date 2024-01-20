#include "timer_dev.h"

static TimerDev_t *TimerDevTable[TIMER_DEV_NUM_MAX];

/*
 * @函数名  TimerDevRegister
 * @用  途  定时器设备注册
 * @参  数  num:注册到的定时器设备号
 *          dev:定时器设备
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevRegister(TIMER_DEV_NUM_t num, TimerDev_t *dev)
{
	DEV_NUM_CHECK(TIMER, num);

	TimerDevTable[num] = dev;

	return TIMER_DEV_OK;
}

/*
 * @函数名  TimerDevUnregister
 * @用  途  定时器设备注销
 * @参  数  num:注销的定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_DEV_NUM_t num)
{
	DEV_NUM_CHECK(TIMER, num);

	TimerDevTable[num] = NULL;

	return TIMER_DEV_OK;
}

/*
 * @函数名  TimerDevInit
 * @用  途  定时器设备初始化
 * @参  数  num:定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevInit(TIMER_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(TIMER, TimerDevTable, num, Init);

	return TimerDevTable[num]->Init();
}

/*
 * @函数名  TimerDevDeInit
 * @用  途  定时器设备反初始化
 * @参  数  num:定时器设备号
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(TIMER, TimerDevTable, num, DeInit);

	return TimerDevTable[num]->DeInit();
}

/*
 * @函数名  TimerDevGetCount
 * @用  途  获取定时器设备的计数值
 * @参  数  num:定时器设备号
 *          count:获取到的设备值
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevGetCount(TIMER_DEV_NUM_t num, uint32_t *count)
{
	DEV_BASE_PARAM_CHECK(TIMER, TimerDevTable, num, GetCount);

	return TimerDevTable[num]->GetCount(count);
}

/*
 * @函数名  TimerDevPWMOut
 * @用  途  定时器设备PWM输出
 * @参  数  num:定时器设备号
 *          channel:PWM输出的通道号
 *          duty:PWM输出值
 * @返回值  错误状态值
*/
TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_DEV_NUM_t num, uint8_t channel, uint32_t duty)
{
	DEV_BASE_PARAM_CHECK(TIMER, TimerDevTable, num, PWMOut);

	return TimerDevTable[num]->PWMOut(channel, duty);
}