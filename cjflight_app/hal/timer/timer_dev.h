#ifndef __TIMER_DEV_H_
#define __TIMER_DEV_H_


#include "common.h"

typedef enum
{
	TIMER_MOTOR_PWM,

	TIMER_MAX
}TIMER_t;


typedef enum
{
	TIMER_DEV_ERROR_OK = 0,
	TIMER_DEV_ERROR_NULL,
	TIMER_DEV_ERROR_INVALID,
}TIMER_DEV_ERROR_t;


typedef struct
{
	TIMER_DEV_ERROR_t (*Init)(void);
	TIMER_DEV_ERROR_t (*DeInit)(void);
	TIMER_DEV_ERROR_t (*PWMOut)(uint8_t channel, uint32_t duty);
}TimerDev_t;

extern TIMER_DEV_ERROR_t TimerDevRegister(TIMER_t timer, TimerDev_t *dev);
extern TIMER_DEV_ERROR_t TimerDevUnregister(TIMER_t timer);
extern TIMER_DEV_ERROR_t TimerDevInit(TIMER_t timer);
extern TIMER_DEV_ERROR_t TimerDevDeInit(TIMER_t timer);
extern TIMER_DEV_ERROR_t TimerDevPWMOut(TIMER_t timer, uint8_t channel, uint32_t duty);

#endif /*__TIMER_DEV_H_*/