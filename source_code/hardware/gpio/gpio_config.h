#ifndef __GPIO_CONFIG_H_
#define __GPIO_CONFIG_H_



#define LED1_GPIO          GPIOC
#define LED1_GPIO_PIN      GPIO_Pins_14

#define LED2_GPIO          GPIOA
#define LED2_GPIO_PIN      GPIO_Pins_10

#define LED3_GPIO          GPIOA
#define LED3_GPIO_PIN      GPIO_Pins_11


#if 1

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





extern void GPIO_ConfigInitAll(void);




#endif /*__GPIO_CONFIG_H_*/
