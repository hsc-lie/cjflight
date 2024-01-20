#include "usart_dev.h"

static USARTDev_t *USARTDevTable[USART_DEV_NUM_MAX];

/*
 * @函数名  USARTDevRegister
 * @用  途  串口设备注册
 * @参  数  num:注册到的串口设备号
 *          dev:串口设备
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevRegister(USART_DEV_NUM_t num, USARTDev_t *dev)
{
	DEV_NUM_CHECK(USART, num);

	USARTDevTable[num] = dev;

	return USART_DEV_OK;
}

/*
 * @函数名  USARTDevUnregister
 * @用  途  串口设备注销
 * @参  数  num:注销的串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevUnregister(USART_DEV_NUM_t num)
{
	DEV_NUM_CHECK(USART, num);

	USARTDevTable[num] = NULL;

	return USART_DEV_OK;
}

/*
 * @函数名  USARTDevInit
 * @用  途  串口设备初始化
 * @参  数  num:串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevInit(USART_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(USART, USARTDevTable, num, Init);

	return USARTDevTable[num]->Init();
}

/*
 * @函数名  USARTDevDeInit
 * @用  途  串口设备反初始化
 * @参  数  num:串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevDeInit(USART_DEV_NUM_t num)
{
	DEV_BASE_PARAM_CHECK(USART, USARTDevTable, num, DeInit);

	return USARTDevTable[num]->DeInit();
}

/*
 * @函数名  USARTDevSendData
 * @用  途  串口设备发送数据
 * @参  数  num:串口设备号
 *          data:发送的数据
 *          len:发送数据的长度
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevSendData(USART_DEV_NUM_t num, uint8_t *data, uint32_t len)
{
	DEV_BASE_PARAM_CHECK(USART, USARTDevTable, num, SendData);

	return USARTDevTable[num]->SendData(data, len);
}

/*
 * @函数名  USARTDevReadData
 * @用  途  串口设备从缓冲区读取数据
 * @参  数  num:串口设备号
 *          data:读取到的数据
 *          len:需要读取数据的长度
 *          outLen:实际读取到的长度
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevReadData(USART_DEV_NUM_t num, uint8_t *data, uint32_t readLen, uint32_t * outLen)
{
	DEV_BASE_PARAM_CHECK(USART, USARTDevTable, num, ReadData);

	return USARTDevTable[num]->ReadData(data, readLen, outLen);
}
