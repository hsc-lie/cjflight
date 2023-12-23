#include "bmp280_cfg.h"

#include "common.h"
/**********************以下根据实际工程使用情况配置**********************************/
#include "i2c_dev.h"

static void BMP280I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevSendData(I2C_TYPE_SENSOR, addr, &reg, 1, data, len);
}

static void BMP280I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevReadData(I2C_TYPE_SENSOR, addr, &reg, 1, data, len);
}


BMP280_t BMP280 = 
{
	.DevAddr = BMP280_ID,

	.I2CWriteReg = BMP280I2CWriteReg,
	.I2CReadReg = BMP280I2CReadReg,
};





