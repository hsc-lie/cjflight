#include "led.h"



void LED_SetValue(LED_t * led, uint32_t value)
{
	if((NULL != led)
		&& (NULL != led->SetValue)
	)
	{
		led->SetValue(value);
	}
}



