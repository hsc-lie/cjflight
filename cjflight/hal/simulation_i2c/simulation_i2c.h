#ifndef __MY_I2C_H
#define __MY_I2C_H


#include "common.h"

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



typedef enum
{
	E_SIMULATION_I2C_ERROR_OK,
	E_SIMULATION_I2C_ERROR_NULL,
	E_SIMULATION_I2C_ERROR_NACK,
	
}E_SIMULATION_I2C_ERROR;


typedef enum
{
	E_SIMULATION_I2C_SDA_RX,
	E_SIMULATION_I2C_SDA_TX,
}E_SIMULATION_I2C_SDA_DIR;



typedef struct
{
	uint32_t DelayCount;

	void (*SCLSet)(uint8_t value);
	void (*SDASet)(uint8_t value);

	void (*SDADirSet)(E_SIMULATION_I2C_SDA_DIR dir);
	uint8_t (*SDARead)(void);

}SimulationI2C_t;



extern E_SIMULATION_I2C_ERROR SimulationI2C_SendData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t *data, uint8_t dataLen);
extern E_SIMULATION_I2C_ERROR SimulationI2C_ReadData(SimulationI2C_t * i2c, uint8_t addr, uint8_t * reg, uint8_t regLen, uint8_t *data, uint8_t dataLen);



#endif


