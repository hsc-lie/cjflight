#include "spl06_cfg.h"

/**********************以下根据实际工程使用情况配置**********************************/

#include "i2c_dev.h"

I2CDev_t *SPL06I2CDev = NULL;


static void SPL06I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevSendData(I2C_DEV_SENSOR, addr, &reg, 1, data, len);
}

static void SPL06I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevReadData(I2C_DEV_SENSOR, addr, &reg, 1, data, len);
}


SPL06_t SPL06 = 
{
	.DevAddr = SPL06_ADDR1,
		
	.PressureRate = SPL06_RATE_64,
	.PressurePRC = SPL06_PRC_TIMES64,
	.TemperatureRate = SPL06_RATE_64,
	.TemperaturePRC = SPL06_PRC_TIMES64,

	.Mode = SPL06_MODE_CONTINUOUS_ALL_MEASUREMENT,
	
	.I2CWriteReg = SPL06I2CWriteReg,
	.I2CReadReg = SPL06I2CReadReg,
};



