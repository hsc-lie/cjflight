#include "my_systick.h"


void systick_delay_us(unsigned int us)
{
	unsigned int i;
	SysTick_Config(120);
	for(i = 0;i < us;i++)
	{
		while(!(SysTick->CTRL & (1<<16)));
	}
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void systick_delay_ms(unsigned int ms)
{
	unsigned int i;
	SysTick_Config(120000);
	for(i = 0;i < ms;i++)
	{
		while(!(SysTick->CTRL & (1<<16)));
	}
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
