#include "bmp280_cfg.h"

#include "common.h"
/**********************以下根据实际工程使用情况配置**********************************/
#include "i2c_dev.h"

I2CDev_t BMP280I2CDev = NULL;

static void BMP280I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	if(NULL == BMP280I2CDev)
	{
		BMP280I2CDev = I2CDevGet(0);
	}

	if (NULL != BMP280I2CDev)
	{
		I2CDevSendData(BMP280I2CDev, addr, &reg, 1, data, len);
	}
}

static void BMP280I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	if(NULL == BMP280I2CDev)
	{
		BMP280I2CDev = I2CDevGet(0);
	}

	if (NULL != BMP280I2CDev)
	{
		I2CDevSendData(BMP280I2CDev, addr, &reg, 1, data, len);
	}
}


BMP280_t BMP280 = 
{
	.DevAddr = BMP280_ID,

	.I2CWriteReg = BMP280I2CWriteReg,
	.I2CReadReg = BMP280I2CReadReg,
};





