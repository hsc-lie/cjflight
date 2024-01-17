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


/*
 * @函数名  I2CDevRegister
 * @用  途  I2C设备注册
 * @参  数  i2c:要注册到的I2C设备号
 *          dev:I2C设备
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevRegister(I2C_t i2c, I2CDev_t *dev)
{
	I2C_DEV_CHECK(i2c);

	I2CDevTable[i2c] = dev;

	return I2C_DEV_ERROR_OK;
}

/*
 * @函数名  I2CDevUnregister
 * @用  途  I2C设备注销
 * @参  数  i2c:要注销的I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevUnregister(I2C_t i2c)
{
	I2C_DEV_CHECK(i2c);

	I2CDevTable[i2c] = NULL;

	return I2C_DEV_ERROR_OK;
}

/*
 * @函数名  I2CDevInit
 * @用  途  I2C设备初始化
 * @参  数  i2c:I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevInit(I2C_t i2c)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->Init);

	return I2CDevTable[i2c]->Init();
}

/*
 * @函数名  I2CDevDeInit
 * @用  途  I2C设备反初始化
 * @参  数  i2c:I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevDeInit(I2C_t i2c)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->DeInit);

	return I2CDevTable[i2c]->DeInit();
}

/*
 * @函数名  I2CDevSendData
 * @用  途  I2C设备发送数据
 * @参  数  i2c:I2C设备号
 *          addr:从机地址
 *          reg:寄存器的值
 *          regLen:寄存器值的长度
 *          data:发送的数据
 *          dataLen:发送数据的长度 
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevSendData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->SendData);
	
	return I2CDevTable[i2c]->SendData(addr, reg, regLen, data, dataLen);
}

/*
 * @函数名  I2CDevReadData
 * @用  途  I2C设备读取数据
 * @参  数  i2c:I2C设备号
 *          addr:从机地址
 *          reg:寄存器的值
 *          regLen:寄存器值的长度
 *          data:读取到的数据
 *          dataLen:需要读取的数据长度 
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevReadData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen)
{
	I2C_DEV_CHECK_FUNC(i2c, I2CDevTable[i2c]->ReadData);

	return I2CDevTable[i2c]->ReadData(addr, reg, regLen, data, dataLen);
}
