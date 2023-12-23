#ifndef _LED_H_
#define _LED_H_


#include "common.h"


typedef struct
{
	void (*SetValue)(uint32_t value);
}LED_t;


void LEDSetValue(LED_t *led, uint32_t value);


#endif  /*_LED_H_*/

