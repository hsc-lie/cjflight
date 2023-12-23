#include "usart_dev.h"



static DoublyListItem_t USARTDevList = 
{
	.Next = &USARTDevList,
	.Prev = &USARTDevList,
};


void USARTDevInit(USARTDev_t * i2c)
{
	DevInit((Dev_t *)i2c);
}

void USARTDevDeInit(USARTDev_t * i2c)
{
	DevDeInit((Dev_t *)i2c);
}

void USARTDevInitAll()
{
	DevInitAll(&USARTDevList);
}

void USARTDevDeInitAll()
{
	DevDeInitAll(&USARTDevList);
}

USARTDev_t *USARTDevGet(uint32_t id)
{
	return (USARTDev_t *)DevGet(&USARTDevList, id);
}

void USARTDevRegister(USARTDev_t *USARTDev)
{
	DevRegister(&USARTDevList, (Dev_t *)USARTDev);
}

void USARTDevUnregister(USARTDev_t *USARTDev)
{
	DevUnregister(&USARTDevList, (Dev_t *)USARTDev);
}


USART_DEV_ERROR_t USARTDevSendData(const USARTDev_t * const usart, uint8_t *data, uint32_t len)
{
	if(NULL == usart->SendData)
	{
		return USART_DEV_ERROR_NULL;
	}

	usart->SendData(data, len);

	return USART_DEV_ERROR_OK;
}


USART_DEV_ERROR_t USARTDevReadData(const USARTDev_t * const usart, uint8_t *data, uint32_t readLen, uint32_t * outLen)
{
	if(NULL == usart->ReadData)
	{
		return USART_DEV_ERROR_NULL;
	}

	usart->ReadData(data, readLen, outLen);

	return USART_DEV_ERROR_OK;
}



