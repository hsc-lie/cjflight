#ifndef __BSP_GPIO_H_
#define __BSP_GPIO_H_

#include "at32f4xx.h"

#define LED0_GPIO                   GPIOC
#define LED0_GPIO_PIN               GPIO_Pins_14
        
#define LED1_GPIO                   GPIOA
#define LED1_GPIO_PIN               GPIO_Pins_10
        
#define LED2_GPIO                   GPIOA
#define LED2_GPIO_PIN               GPIO_Pins_11

#define DEBUG_USART_TX_GPIO         GPIOB
#define DEBUG_USART_TX_PIN          GPIO_Pins_6
#define DEBUG_USART_TX_AF           GPIO_AF_0
#define DEBUG_USART_TX_PIN_SOURCE   GPIO_PinsSource6
      
#define DEBUG_USART_RX_GPIO         GPIOA
#define DEBUG_USART_RX_PIN          GPIO_Pins_7
#define DEBUG_USART_RX_AF           GPIO_AF_0
#define DEBUG_USART_RX_PIN_SOURCE   GPIO_PinsSource7
      
#define IBUS_USART_RX_GPIO          GPIOA
#define IBUS_USART_RX_PIN           GPIO_Pins_3
#define IBUS_USART_RX_AF            GPIO_AF_1
#define IBUS_USART_RX_PIN_SOURCE    GPIO_PinsSource3




#if 0//四旋翼上
#define SIMULATION_I2C_SCL_GPIO          GPIOA
#define SIMULATION_I2C_SCL_GPIO_PIN      GPIO_Pins_10
#define SIMULATION_I2C_SDA_GPIO          GPIOA
#define SIMULATION_I2C_SDA_GPIO_PIN      GPIO_Pins_9
#else//核心板上
#define SIMULATION_I2C_SCL_GPIO          GPIOA
#define SIMULATION_I2C_SCL_GPIO_PIN      GPIO_Pins_8
#define SIMULATION_I2C_SDA_GPIO          GPIOB
#define SIMULATION_I2C_SDA_GPIO_PIN      GPIO_Pins_15
#endif

extern void BSPGPIOInitAll(void);

#endif /*__BSP_GPIO_H_*/
