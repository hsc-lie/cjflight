#ifndef _UART_H_
#define _UART_H_

#include "stdio.h"

#include "at32f4xx.h"
#include "at32f4xx_gpio_ex.h"
#include "at32f4xx_usart.h"


#define UART_BAUDRATE            115200


#define UART_TX_GPIO_RCC         RCC_AHBPERIPH_GPIOB   
#define UART_RX_GPIO_RCC         RCC_AHBPERIPH_GPIOB  

#define UART_RCC                 RCC_APB2PERIPH_USART1

#define UARTx                     USART1

#define UART_TX_GPIO             GPIOB
#define UART_RX_GPIO             GPIOB

#define UART_TX_GPIO_PIN            GPIO_Pins_6
#define UART_RX_GPIO_PIN            GPIO_Pins_7


#define UART_TX_GPIO_PINSSOURCE            GPIO_PinsSource6
#define UART_RX_GPIO_PINSSOURCE            GPIO_PinsSource7

#define UART_TX_GPIO_AF                    GPIO_AF_0
#define UART_RX_GPIO_AF                    GPIO_AF_0


#define UART_IRQ                           USART1_IRQn



void uart_init(void);
void uart_send_byte(USART_Type * UART,uint8_t data);
int fputc(int ch, FILE *f);


#endif /*_UART_H_*/

