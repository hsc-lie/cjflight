#include "led_cfg.h"


/*****************************以下所有代码按工程实际需要配置*******************************/
#include "gpio_dev.h"


static void LED0SetValue(uint32_t value)
{
	GPIODevWritePinOut(GPIO_TYPE_LED0, value);
}

static void LED1SetValue(uint32_t value)
{
	GPIODevWritePinOut(GPIO_TYPE_LED1, value);
}

static void LED2SetValue(uint32_t value)
{
	GPIODevWritePinOut(GPIO_TYPE_LED2, value);
}


LED_t LED[3] = 
{
	{.SetValue = LED0SetValue},
	{.SetValue = LED1SetValue},
	{.SetValue = LED2SetValue},
};

