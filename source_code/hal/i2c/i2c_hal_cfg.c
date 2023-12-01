#include "i2c_hal_cfg.h"

#include "simulation_i2c_cfg.h" 





static int I2C1_SendData(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2C_SendData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}

static int I2C1_ReadData(uint8_t       addr, uint8_t *reg, uint32_t regLen, uint8_t * data, uint32_t dataLen)
{
	return SimulationI2C_ReadData(&SimulationI2C1, addr, reg, regLen, data, dataLen);
}


I2C_HAL_t I2C_Dev1 = 
{
	.Init = NULL,
	.SendData = I2C1_SendData,
	.ReadData = I2C1_ReadData,
};







