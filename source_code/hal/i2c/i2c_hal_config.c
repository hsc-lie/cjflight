#include "i2c_hal_config.h"






static int I2C1_SendData(uint8_t addr, uint8_t * data, uint32_t len, uint8_t isSendStop)
{
	return 1;
}

int I2C1_ReadData(uint8_t      addr, uint8_t * data, uint32_t len)
{
	return 1;
}


I2C_HAL_t I2C_Dev1 = 
{
	.Init = NULL,
	.SendData = I2C1_SendData,
	.ReadData = I2C1_ReadData,
};







