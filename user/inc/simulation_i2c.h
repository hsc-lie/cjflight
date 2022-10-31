#ifndef __MY_I2C_H
#define __MY_I2C_H

#include "at32f4xx.h"



#if 1

#define I2C_GPIO_CLK             RCC_AHBPERIPH_GPIOA

#define I2C_GPIO_SCL_PROT             GPIOA
#define I2C_SCL_PIN                   GPIO_Pins_10


#define I2C_GPIO_SDA_PROT             GPIOA
#define I2C_SDA_PIN                   GPIO_Pins_9

#else

#define I2C_GPIO_CLK             RCC_AHBPERIPH_GPIOA | RCC_AHBPERIPH_GPIOB

#define I2C_GPIO_SCL_PROT             GPIOA
#define I2C_SCL_PIN                   GPIO_Pins_8


#define I2C_GPIO_SDA_PROT             GPIOB
#define I2C_SDA_PIN                   GPIO_Pins_15

#endif





#define i2c_scl_1()                GPIO_SetBits(I2C_GPIO_SCL_PROT, I2C_SCL_PIN)
#define i2c_scl_0()                GPIO_ResetBits(I2C_GPIO_SCL_PROT, I2C_SCL_PIN)

#define i2c_sda_1()                GPIO_SetBits(I2C_GPIO_SDA_PROT, I2C_SDA_PIN)
#define i2c_sda_0()                GPIO_ResetBits(I2C_GPIO_SDA_PROT, I2C_SDA_PIN)


#define i2c_sda_read()             GPIO_ReadInputDataBit(I2C_GPIO_SDA_PROT, I2C_SDA_PIN)






void simulation_i2c_init(void);
int simulation_i2c_writereg(uint8_t addr, uint8_t reg ,uint8_t data);
int simulation_i2c_writeregs(uint8_t addr, uint8_t reg ,uint8_t len,uint8_t *data);
int simulation_i2c_readregs(uint8_t addr, uint8_t reg ,uint8_t len,uint8_t *data);

#endif


