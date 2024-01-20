#ifndef __BSP_TIMER_H_
#define __BSP_TIMER_H_

#include "common.h"
#include "at32f4xx.h"

extern void BSPTimer3Init(void);
extern void BSPTimer6Init(void);
extern uint32_t TimerGetCount(TMR_Type* TMRx);

#endif /*__BSP_TIMER_H_*/
