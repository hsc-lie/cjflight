#include "bmp280_cfg.h"



/**********************以下根据实际工程使用情况配置**********************************/
#include "i2c_hal_cfg.h"


static void BMP280_I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalSendData(&I2C_Dev1, addr, &reg, 1, data, len);
}

static void BMP280_I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalReadData(&I2C_Dev1, addr, &reg, 1, data, len);
}


BMP280_t BMP280 = 
{
	.DevAddr = BMP280_ID,

	.I2CWriteReg = BMP280_I2CWriteReg,
	.I2CReadReg = BMP280_I2CReadReg,
};





