#include "led.h"


void LEDSetValue(LED_t *led, uint32_t value)
{
	if(NULL != led->SetValue)
	{
		led->SetValue(value);
	}
}



