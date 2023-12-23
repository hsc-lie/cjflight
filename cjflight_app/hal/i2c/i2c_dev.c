#include "i2c_dev.h"


#define I2C_DEV_CHECK(i2c) \
if((i2c) >= I2C_MAX)\
{\
	return I2C_DEV_ERROR_INVALID;\
}\
while(0)

#define I2C_DEV_CHECK_FUNC(i2c, func) \
I2C_DEV_CHECK(i2c);\
if((NULL == I2CDevTable[i2c]) || (NULL == (func)))\
{\
	return I2C_DEV_ERROR_NULL;\
}\
while(0)

static I2CDev_t *I2CDevTable[I2C_MAX];


I2C_DEV_ERROR_t I2CDevRegister(I2C_t i2c, I2CDev_t *dev)
{
	I2C_DEV_CHECK(i2c);

	I2CDevTable[i2c] = dev;

	return I2C_DEV_ERROR_OK;
}

I2C_DEV_ERROR_t I2CDevUnregister(I2C_t i2c)
{
	I2C_DEV_CHECK(i2c);

	I2CDevTable[i2c] = NULL;

	return I2C_DEV_ERROR_OK;
}


I2C_DEV_ERROR_t I2CDevInit(I2C_t i2c)
{
	I2C_DEV_ERROR_t ret;

	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->Init);

	return I2CDevTable[i2c]->Init();
}

I2C_DEV_ERROR_t I2CDevDeInit(I2C_t i2c)
{
	I2C_DEV_ERROR_t ret;

	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->DeInit);

	return I2CDevTable[i2c]->DeInit();
}


I2C_DEV_ERROR_t I2CDevSendData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->SendData);
	
	return I2CDevTable[i2c]->SendData(addr, reg, regLen, data, dataLen);
}


I2C_DEV_ERROR_t I2CDevReadData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->ReadData);

	return I2CDevTable[i2c]->ReadData(addr, reg, regLen, data, dataLen);
}

