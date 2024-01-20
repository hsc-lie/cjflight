#ifndef __TIMER_DEV_H_
#define __TIMER_DEV_H_

#include "dev.h"

typedef enum
{
	TIMER_DEV_TEST,
	TIMER_DEV_MOTOR_PWM,
	TIMER_DEV_NUM_MAX
}TIMER_DEV_NUM_t;

typedef enum
{
	TIMER_DEV_OK = 0,                             //成功
	TIMER_DEV_ERROR_INVALID_PARAM,                //无效的参数
	TIMER_DEV_ERROR_UNREGISTERED,                 //设备未注册
	TIMER_DEV_ERROR_NULL_FUNC,                    //空的函数
}TIMER_DEV_ERROR_t;

typedef struct
{
	TIMER_DEV_ERROR_t (*Init)(void);
	TIMER_DEV_ERROR_t (*DeInit)(void);
	TIMER_DEV_ERROR_t (*GetCount)(uint32_t *count);
	TIMER_DEV_ERROR_t (*PWMOut)(uint8_t channel, uint32_t duty);
}TimerDev_t;

extern TIMER_DEV_ERROR_t TimerDevRegister(TIMER_DEV_NUM_t num, TimerDev_t *dev);
extern TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_DEV_NUM_t num);
extern TIMER_DEV_ERROR_t TimerDevInit(TIMER_DEV_NUM_t num);
extern TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_DEV_NUM_t num);
extern TIMER_DEV_ERROR_t TimerDevGetCount(TIMER_DEV_NUM_t num, uint32_t *count);
extern TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_DEV_NUM_t num, uint8_t channel, uint32_t duty);

#endif /*__TIMER_DEV_H_*/