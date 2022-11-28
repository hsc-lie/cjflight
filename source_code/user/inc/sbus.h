#ifndef _SBUS_H_
#define _SBUS_H_


#include "at32f4xx.h"

#include "common.h"

#include "at32f4xx_gpio_ex.h"
#include "at32f4xx_usart.h"


#define SBUS_UART_BAUDRATE            115200 //100000

#define SBUS_UART_RX_GPIO_RCC         RCC_AHBPERIPH_GPIOA  

#define SBUS_UART_RCC                 RCC_APB1PERIPH_USART2

#define SBUS_UARTx                     USART2

#define SBUS_UART_RX_GPIO             GPIOA

#define SBUS_UART_RX_GPIO_PIN            GPIO_Pins_3

#define SBUS_UART_RX_GPIO_PINSSOURCE            GPIO_PinsSource3

#define SBUS_UART_RX_GPIO_AF                    GPIO_AF_1


#define SBUS_UART_IRQ                           USART2_IRQn


#ifndef IBUS_Channel_Type
typedef enum
{
    IBUS_Channel1,
    IBUS_Channel2,
    IBUS_Channel3,
    IBUS_Channel4,
    IBUS_Channel5,
    IBUS_Channel6,
    IBUS_Channel7,
    IBUS_Channel8,
    IBUS_Channel_MAX
}IBUS_Channel_Type;
#endif



typedef struct
{

    uint8_t data_buff[32];
    uint16_t data_sum;
}IBus_data_t;


typedef struct
{
    uint8_t data_read_index;
    uint8_t data_read_flag;
    IBus_data_t data[2];
}IBus_handel_t;


extern uint16_t ibus_solution_data[IBUS_Channel_MAX];

void sbus_init(void);
void sbus_updata_data(void);


//void ibus_read_original_data(void);
//void ibus_read_original_data(uint8_t data);
extern int IBUS_Analysis();


uint8_t ibus_data_solution(void);  
uint16_t ibus_get_channel_value(IBUS_Channel_Type Channel);


#endif  /*_SBUS_H_*/
