#include "usart_dev.h"

static USARTDev_t *USARTDevTable[USART_MAX];


#define USART_DEV_CHECK(usart) \
if(usart >= USART_MAX)\
{\
	return USART_DEV_ERROR_INVALID;\
}\
while(0)

#define USART_DEV_CHECK_FUNC(usart, func) \
USART_DEV_CHECK(usart)\
if((NULL == USARTDevTable[usart]) || (NULL == func))\
{\
	return USART_DEV_ERROR_NULL;\
}\
while(0)

USART_DEV_ERROR_t USARTDevRegister(USART_t usart, USARTDev_t *dev)
{
	USART_DEV_CHECK(usart);

	USARTDevTable[usart] = dev;

	return USART_DEV_ERROR_OK;
}

USART_DEV_ERROR_t USARTDevUnregister(USART_t usart)
{
	USART_DEV_CHECK(usart);

	USARTDevTable[usart] = NULL;

	return USART_DEV_ERROR_OK;
}


USART_DEV_ERROR_t USARTDevInit(USART_t usart)
{
	USART_DEV_ERROR_t ret;

	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->Init);

	return USARTDevTable[usart]->Init();
}

USART_DEV_ERROR_t USARTDevDeInit(USART_t usart)
{
	USART_DEV_ERROR_t ret;

	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->DeInit);

	return USARTDevTable[usart]->DeInit();
}


USART_DEV_ERROR_t USARTDevSendData(USART_t usart, uint8_t *data, uint32_t len)
{
	USART_DEV_ERROR_t ret;

	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->SendData);

	return USARTDevTable[usart]->SendData(data, len);
}


USART_DEV_ERROR_t USARTDevReadData(USART_t usart, uint8_t *data, uint32_t readLen, uint32_t * outLen)
{
	USART_DEV_ERROR_t ret;

	USART_DEV_CHECK_FUNC(usart, USARTDevTable[usart]->ReadData);

	return USARTDevTable[usart]->ReadData(data, readLen, outLen);
}



