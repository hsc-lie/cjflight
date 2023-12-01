#ifndef _LED_H_
#define _LED_H_


#include "common.h"


typedef struct
{
	void (*SetValue)(uint32_t value);
}LED_t;




/*
#define LED1_ON()            GPIO_ResetBits(LED1_GPIO, LED1_GPIO_PIN)
#define LED1_OFF()           GPIO_SetBits(LED1_GPIO, LED1_GPIO_PIN)

#define LED2_ON()            GPIO_ResetBits(LED2_GPIO, LED2_GPIO_PIN)
#define LED2_OFF()           GPIO_SetBits(LED2_GPIO, LED2_GPIO_PIN)

#define LED3_ON()            GPIO_ResetBits(LED3_GPIO, LED3_GPIO_PIN)
#define LED3_OFF()           GPIO_SetBits(LED3_GPIO, LED3_GPIO_PIN)
*/


void LED_SetValue(LED_t * led, uint32_t value);


#endif  /*_LED_H_*/

