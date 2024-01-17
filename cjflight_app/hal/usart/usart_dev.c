#include "usart_dev.h"

#define USART_DEV_CHECK(usart) \
if(usart >= USART_MAX)\
{\
	return USART_DEV_ERROR_INVALID;\
}\
while(0)

#define USART_DEV_CHECK_FUNC(usart, func) \
USART_DEV_CHECK(usart);\
if((NULL == USARTDevTable[usart]) || (NULL == func))\
{\
	return USART_DEV_ERROR_NULL;\
}\
while(0)


static USARTDev_t *USARTDevTable[USART_MAX];


/*
 * @函数名  USARTDevRegister
 * @用  途  串口设备注册
 * @参  数  usart:注册到的串口设备号
 *          dev:串口设备
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevRegister(USART_t usart, USARTDev_t *dev)
{
	USART_DEV_CHECK(usart);

	USARTDevTable[usart] = dev;

	return USART_DEV_ERROR_OK;
}

/*
 * @函数名  USARTDevUnregister
 * @用  途  串口设备注销
 * @参  数  usart:注销的串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevUnregister(USART_t usart)
{
	USART_DEV_CHECK(usart);

	USARTDevTable[usart] = NULL;

	return USART_DEV_ERROR_OK;
}

/*
 * @函数名  USARTDevInit
 * @用  途  串口设备初始化
 * @参  数  usart:串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevInit(USART_t usart)
{
	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->Init);

	return USARTDevTable[usart]->Init();
}

/*
 * @函数名  USARTDevDeInit
 * @用  途  串口设备反初始化
 * @参  数  usart:串口设备号
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevDeInit(USART_t usart)
{
	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->DeInit);

	return USARTDevTable[usart]->DeInit();
}

/*
 * @函数名  USARTDevSendData
 * @用  途  串口设备发送数据
 * @参  数  usart:串口设备号
 *          data:发送的数据
 *          len:发送数据的长度
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevSendData(USART_t usart, uint8_t *data, uint32_t len)
{
	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->SendData);

	return USARTDevTable[usart]->SendData(data, len);
}

/*
 * @函数名  USARTDevReadData
 * @用  途  串口设备从缓冲区读取数据
 * @参  数  usart:串口设备号
 *          data:读取到的数据
 *          len:需要读取数据的长度
 *          outLen:实际读取到的长度
 * @返回值  错误状态值
*/
USART_DEV_ERROR_t USARTDevReadData(USART_t usart, uint8_t *data, uint32_t readLen, uint32_t * outLen)
{
	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->ReadData);

	return USARTDevTable[usart]->ReadData(data, readLen, outLen);
}
