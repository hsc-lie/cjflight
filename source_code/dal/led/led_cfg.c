#include "led_cfg.h"


/*****************************以下所有代码按工程实际需要配置*******************************/
#include "gpio_cfg.h"

#define LED1_ON()            GPIO_ResetBits(LED1_GPIO, LED1_GPIO_PIN)
#define LED1_OFF()           GPIO_SetBits(LED1_GPIO, LED1_GPIO_PIN)

#define LED2_ON()            GPIO_ResetBits(LED2_GPIO, LED2_GPIO_PIN)
#define LED2_OFF()           GPIO_SetBits(LED2_GPIO, LED2_GPIO_PIN)

#define LED3_ON()            GPIO_ResetBits(LED3_GPIO, LED3_GPIO_PIN)
#define LED3_OFF()           GPIO_SetBits(LED3_GPIO, LED3_GPIO_PIN)


static void LED1_SetValue(uint32_t value)
{
	if(0 == value)
	{
		GPIO_SetBits(LED1_GPIO, LED1_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(LED1_GPIO, LED1_GPIO_PIN);
	}
}

static void LED2_SetValue(uint32_t value)
{
	if(0 == value)
	{
		GPIO_SetBits(LED2_GPIO, LED2_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(LED2_GPIO, LED2_GPIO_PIN);
	}
}

static void LED3_SetValue(uint32_t value)
{
	if(0 == value)
	{
		GPIO_SetBits(LED3_GPIO, LED3_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(LED3_GPIO, LED3_GPIO_PIN);
	}
}





LED_t LED1 = 
{
	.SetValue = LED1_SetValue,
};

LED_t LED2 = 
{
	.SetValue = LED2_SetValue,
};

LED_t LED3 = 
{
	.SetValue = LED3_SetValue,
};




