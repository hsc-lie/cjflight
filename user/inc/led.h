#ifndef _LED_H_
#define _LED_H_


#include "at32f4xx.h"
#include "at32f4xx_gpio_ex.h"


typedef enum
{
    LED1_ON,
    LED2_ON,
    LED3_ON,
    LED_Static_Max
}LED_Static_Type;



#define LED1_GPIO_RCC_CLK   RCC_AHBPERIPH_GPIOC
#define LED2_GPIO_RCC_CLK   RCC_AHBPERIPH_GPIOA
#define LED3_GPIO_RCC_CLK   RCC_AHBPERIPH_GPIOA

#define LED1_GPIO          GPIOC
#define LED2_GPIO          GPIOA
#define LED3_GPIO          GPIOA


#define LED1_GPIO_PIN      GPIO_Pins_14
#define LED2_GPIO_PIN      GPIO_Pins_10
#define LED3_GPIO_PIN      GPIO_Pins_11


#define LED1_ON()            GPIO_ResetBits(LED1_GPIO, LED1_GPIO_PIN)
#define LED1_OFF()           GPIO_SetBits(LED1_GPIO, LED1_GPIO_PIN)

#define LED2_ON()            GPIO_ResetBits(LED2_GPIO, LED2_GPIO_PIN)
#define LED2_OFF()           GPIO_SetBits(LED2_GPIO, LED2_GPIO_PIN)

#define LED3_ON()            GPIO_ResetBits(LED3_GPIO, LED3_GPIO_PIN)
#define LED3_OFF()           GPIO_SetBits(LED3_GPIO, LED3_GPIO_PIN)


void led_init(void);
void led_water_lamp(void);

#endif  /*_LED_H_*/

