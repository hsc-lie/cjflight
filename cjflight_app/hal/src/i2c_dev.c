#include "i2c_dev.h"

static I2CDev_t *I2CDevTable[I2C_DEV_NUM_MAX];

/*
 * @函数名  I2CDevRegister
 * @用  途  I2C设备注册
 * @参  数  num:要注册到的I2C设备号
 *          dev:I2C设备
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevRegister(I2C_DEV_NUM_t num, I2CDev_t *dev)
{
	DEV_NUM_CHECK(I2C, num);

	I2CDevTable[num] = dev;

	return I2C_DEV_OK;
}

/*
 * @函数名  I2CDevUnregister
 * @用  途  I2C设备注销
 * @参  数  num:要注销的I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevUnregister(I2C_DEV_NUM_t num)
{
	DEV_NUM_CHECK(I2C, num);

	I2CDevTable[num] = NULL;

	return I2C_DEV_OK;
}

/*
 * @函数名  I2CDevInit
 * @用  途  I2C设备初始化
 * @参  数  num:I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevInit(I2C_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(I2C, I2CDevTable, num, Init);

	return I2CDevTable[num]->Init();
}

/*
 * @函数名  I2CDevDeInit
 * @用  途  I2C设备反初始化
 * @参  数  num:I2C设备号
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevDeInit(I2C_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(I2C, I2CDevTable, num, DeInit);

	return I2CDevTable[num]->DeInit();
}

/*
 * @函数名  I2CDevSendData
 * @用  途  I2C设备发送数据
 * @参  数  num:I2C设备号
 *          addr:从机地址
 *          reg:寄存器的值
 *          regLen:寄存器值的长度
 *          data:发送的数据
 *          dataLen:发送数据的长度 
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevSendData(I2C_DEV_NUM_t num, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen)
{
	DEV_BASE_PARAM_CHECK(I2C, I2CDevTable, num, SendData);
	
	return I2CDevTable[num]->SendData(addr, reg, regLen, data, dataLen);
}

/*
 * @函数名  I2CDevReadData
 * @用  途  I2C设备读取数据
 * @参  数  num:I2C设备号
 *          addr:从机地址
 *          reg:寄存器的值
 *          regLen:寄存器值的长度
 *          data:读取到的数据
 *          dataLen:需要读取的数据长度 
 * @返回值  错误状态值
*/
I2C_DEV_ERROR_t I2CDevReadData(I2C_DEV_NUM_t num, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen)
{
	DEV_BASE_PARAM_CHECK(I2C, I2CDevTable, num, ReadData);

	return I2CDevTable[num]->ReadData(addr, reg, regLen, data, dataLen);
}
