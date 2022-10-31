#include "led.h"


void led_init()
{
    GPIO_InitType GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(LED1_GPIO_RCC_CLK | LED2_GPIO_RCC_CLK | LED3_GPIO_RCC_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pins  = LED1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
    GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;
    GPIO_Init(LED1_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins  = LED2_GPIO_PIN;
    GPIO_Init(LED2_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pins  = LED3_GPIO_PIN;
    GPIO_Init(LED3_GPIO, &GPIO_InitStructure);

	LED1_ON();
    LED2_OFF();
    LED3_OFF();

}


void led_water_lamp()
{
    static led_static = 0;
    led_static++;
    if(LED_Static_Max == led_static)
    {
        led_static = LED1_ON;
    }
    switch(led_static)
    {
        case LED1_ON:
        {
            LED1_ON();
            LED2_OFF();
            LED3_OFF();
            break;
        }
        case LED2_ON:
        {
            LED1_OFF();
            LED2_ON();
            LED3_OFF();
            break;
        }
        case LED3_ON:
        {
            LED1_OFF();
            LED2_OFF();
            LED3_ON();
            break;
        }
        default:
            break;
    }

}


