#include "spl06_cfg.h"


/**********************以下根据实际工程使用情况配置**********************************/
#include "i2c_hal_config.h"


static void SPL06_I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalSendData(&I2C_Dev1, addr, &reg, 1, data, len);
}

static void SPL06_I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalReadData(&I2C_Dev1, addr, &reg, 1, data, len);
}


SPL06_t SPL06 = 
{
	.DevAddr = SPL06_ADDR1,
		
	.PressureRate = E_SPL06_RATE_64,
	.PressurePRC = E_SPL06_PRC_TIMES64,
	.TemperatureRate = E_SPL06_RATE_64,
	.TemperaturePRC = E_SPL06_PRC_TIMES64,

	.Mode = E_SPL06_MODE_CONTINUOUS_ALL_MEASUREMENT,
	
	.I2CWriteReg = SPL06_I2CWriteReg,
	.I2CReadReg = SPL06_I2CReadReg,
};



