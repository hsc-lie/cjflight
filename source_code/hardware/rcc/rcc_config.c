#include "rcc_config.h"
#include "at32f4xx.h"


void RCC_ConfigInitAll()
{
	//GPIO时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB | RCC_AHBPERIPH_GPIOC | RCC_AHBPERIPH_GPIOF, ENABLE);

	//串口1时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE); 
}





