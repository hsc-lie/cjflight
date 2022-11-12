#include "i2c_hal.h"


E_I2C_ERROR I2C_HalInit(I2C_HAL_t * i2c)
{
	if(NULL != i2c->Init)
	{
		i2c->Init();
	}
}

E_I2C_ERROR I2C_HalSendData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * data, uint32_t len, uint8_t isSendStop)
{
	if(NULL != i2c->SendData)
	{
		return i2c->SendData(addr, data, len, isSendStop);
	}
	else
	{
		return E_I2C_ERROR_NULL;
	}
	
}

E_I2C_ERROR I2C_HalReadData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * data, uint32_t len)
{
	if(NULL != i2c->ReadData)
	{
		return i2c->ReadData(addr, data, len);
	}
	else
	{
		return E_I2C_ERROR_NULL;
	}
	
}


E_I2C_ERROR I2C_HalReadRegData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen)
{
	if((NULL == i2c->SendData)
		|| (NULL == i2c->ReadData)
	)
	{
		return E_I2C_ERROR_NULL;
	}

	i2c->SendData(addr, reg, regLen, FALSE);

	i2c->ReadData(addr, data, dataLen);

	return E_I2C_ERROR_OK;
}




