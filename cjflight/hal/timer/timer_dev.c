#include "timer_dev.h"



static DoublyListItem_t TimerDevList = 
{
	.Next = &TimerDevList,
	.Prev = &TimerDevList,
};


void TimerDevInit(TimerDev_t * i2c)
{
	DevInit((Dev_t *)i2c);
}

void TimerDevDeInit(TimerDev_t * i2c)
{
	DevDeInit((Dev_t *)i2c);
}

void TimerDevInitAll()
{
	DevInitAll(&TimerDevList);
}

void TimerDevDeInitAll()
{
	DevDeInitAll(&TimerDevList);
}

TimerDev_t *TimerDevGet(uint32_t id)
{
	return (TimerDev_t *)DevGet(&TimerDevList, id);
}

void TimerDevRegister(TimerDev_t *TimerDev)
{
	DevRegister(&TimerDevList, (Dev_t *)TimerDev);
}

void TimerDevUnregister(TimerDev_t *TimerDev)
{
	DevUnregister(&TimerDevList, (Dev_t *)TimerDev);
}


void TimerPWMOut(TimerDev_t * timer, uint8_t channel, uint32_t duty)
{
	if(NULL != timer)
	{
		timer->PWMOut(channel, duty);
	}
}