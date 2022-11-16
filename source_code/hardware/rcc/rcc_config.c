#include "rcc_config.h"
#include "at32f4xx.h"


void RCC_ConfigInitAll()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB | RCC_AHBPERIPH_GPIOC | RCC_AHBPERIPH_GPIOF, ENABLE);
}





