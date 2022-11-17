#include "simulation_i2c_config.h"

#include "at32f4xx.h"
#include "at32f4xx_gpio_ex.h"


#define SIMULATION_I2C_DELAY_COUNT            (100)

#define SIMULATION_I2C1_SCL_GPIOX         GPIOA
#define SIMULATION_I2C1_SCL_PIN           GPIO_Pins_10

#define SIMULATION_I2C1_SDA_GPIOX         GPIOA
#define SIMULATION_I2C1_SDA_PIN           GPIO_Pins_9



static void SimulationI2C1_SCLSet(uint8_t value)
{
	if(0 == value)
	{
	 	GPIO_ResetBits(SIMULATION_I2C1_SCL_GPIOX, SIMULATION_I2C1_SCL_PIN);
	}
	else
	{
		GPIO_SetBits(SIMULATION_I2C1_SCL_GPIOX, SIMULATION_I2C1_SCL_PIN);
	}
}

static void SimulationI2C1_SDASet(uint8_t value)
{
	if(0 == value)
	{
	 	GPIO_ResetBits(SIMULATION_I2C1_SDA_GPIOX, SIMULATION_I2C1_SDA_PIN);
	}
	else
	{
		GPIO_SetBits(SIMULATION_I2C1_SDA_GPIOX, SIMULATION_I2C1_SDA_PIN);
	}
}

static void SimulationI2C1_SDADirSet(E_SIMULATION_I2C_SDA_DIR dir)
{
	if(E_SIMULATION_I2C_SDA_RX == dir)
	{
		GPIO_SetBits(SIMULATION_I2C1_SDA_GPIOX, SIMULATION_I2C1_SDA_PIN);
	}
}

static uint8_t SimulationI2C1_SDARead(uint8_t value)
{
	return GPIO_ReadInputDataBit(SIMULATION_I2C1_SDA_GPIOX, SIMULATION_I2C1_SDA_PIN);
}




SimulationI2C_t SimulationI2C1 = 
{
	.DelayCount = SIMULATION_I2C_DELAY_COUNT,
	.SCLSet = SimulationI2C1_SCLSet,
	.SDASet = SimulationI2C1_SDASet,
	.SDADirSet = SimulationI2C1_SDADirSet,
	.SDARead = SimulationI2C1_SDARead,
};





