#include "i2c_dev.h"



static DoublyListItem_t I2CDevList = 
{
	.Next = &I2CDevList,
	.Prev = &I2CDevList,
};


void I2CDevInit(I2CDev_t *i2c)
{
	DevInit((Dev_t *)i2c);
}

void I2CDevDeInit(I2CDev_t *i2c)
{
	DevDeInit((Dev_t *)i2c);
}

void I2CDevInitAll()
{
	DevInitAll(&I2CDevList);
}

void I2CDevDeInitAll()
{
	DevDeInitAll(&I2CDevList);
}


I2CDev_t *I2CDevGet(uint32_t id)
{
	return (I2CDev_t *)DevGet(&I2CDevList, id);
}


void I2CDevRegister(I2CDev_t *i2c)
{
	DevRegister(&I2CDevList, (Dev_t *)i2c);
}

void I2CDevUnregister(I2CDev_t *i2c)
{
	DevUnregister(&I2CDevList, (Dev_t *)i2c);
}




I2C_ERROR_t I2CDevSendData(I2CDev_t *i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen)
{
	if(NULL == i2c->SendData)
	{
		return I2C_ERROR_NULL;
	}

	return i2c->SendData(addr, reg, regLen, data, dataLen);
	
}


I2C_ERROR_t I2CDevReadData(I2CDev_t *i2c, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen)
{
	if(NULL == i2c->ReadData)
	{
		return I2C_ERROR_NULL;
	}

	i2c->ReadData(addr, reg, regLen, data, dataLen);

	return I2C_ERROR_OK;
}




