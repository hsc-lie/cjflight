#ifndef __BSP_GPIO_H_
#define __BSP_GPIO_H_

#include "at32f4xx.h"

#define LED0_GPIO          GPIOC
#define LED0_GPIO_PIN      GPIO_Pins_14

#define LED1_GPIO          GPIOA
#define LED1_GPIO_PIN      GPIO_Pins_10

#define LED2_GPIO          GPIOA
#define LED2_GPIO_PIN      GPIO_Pins_11


#if 0

#define SIMULATION_I2C_SCL_GPIO          GPIOA
#define SIMULATION_I2C_SCL_GPIO_PIN      GPIO_Pins_10

#define SIMULATION_I2C_SDA_GPIO          GPIOA
#define SIMULATION_I2C_SDA_GPIO_PIN      GPIO_Pins_9

#else

#define SIMULATION_I2C_SCL_GPIO          GPIOA
#define SIMULATION_I2C_SCL_GPIO_PIN      GPIO_Pins_8

#define SIMULATION_I2C_SDA_GPIO          GPIOB
#define SIMULATION_I2C_SDA_GPIO_PIN      GPIO_Pins_15


#endif

extern void BSPGPIOInitAll(void);

#endif /*__BSP_GPIO_H_*/
