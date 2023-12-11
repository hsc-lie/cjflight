#include "bsp_rcc.h"
#include "at32f4xx.h"


void BSPRCCInitAll()
{
	//GPIO时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB | RCC_AHBPERIPH_GPIOC | RCC_AHBPERIPH_GPIOF, ENABLE);

	//DMA1时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);
	

	//串口1时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_USART1, ENABLE); 
	
	//串口2时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART2, ENABLE);

	//Timer3时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_TMR3, ENABLE);
	
}





