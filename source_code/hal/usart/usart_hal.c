#include "usart_hal.h"


E_USART_HAL_ERROR USART_HAL_SendData(const USART_HAL_t const * usart, uint8_t *data, uint32_t len)
{
	if((NULL == usart)
		|| (NULL == usart->SendData)
	)
	{
		return E_USART_HAL_ERROR_NULL;
	}

	usart->SendData(data, len);

	return E_USART_HAL_ERROR_OK;
}


E_USART_HAL_ERROR USART_HAL_ReadData(const USART_HAL_t const * usart, uint8_t *data, uint32_t readLen, uint32_t * outLen)
{
	if((NULL == usart)
		|| (NULL == usart->ReadData)
	)
	{
		return E_USART_HAL_ERROR_NULL;
	}

	usart->ReadData(data, readLen, outLen);

	return E_USART_HAL_ERROR_OK;
}



