#ifndef __TIMER_DEV_H_
#define __TIMER_DEV_H_



#include "dev.h"

typedef struct
{
	Dev_t Dev;

	void (*PWMOut)(uint8_t channel, uint32_t duty);
}TimerDev_t;


extern void TimerDevInit(TimerDev_t * i2c);
extern void TimerDevDeInit(TimerDev_t * i2c);
extern void TimerDevInitAll();
extern void TimerDevDeInitAll();
extern TimerDev_t *TimerDevGet(uint32_t id);
extern void TimerDevRegister(TimerDev_t *TimerDev);
extern void TimerDevUnregister(TimerDev_t *TimerDev);

extern void TimerPWMOut(TimerDev_t * timer, uint8_t channel, uint32_t duty);

#endif /*__TIMER_DEV_H_*/