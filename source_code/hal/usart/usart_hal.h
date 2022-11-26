#ifndef __USART_HAL_H_
#define __USART_HAL_H_

#include "common.h"


typedef enum
{
	E_USART_HAL_ERROR_OK,
	E_USART_HAL_ERROR_NULL,
}E_USART_HAL_ERROR;


typedef struct
{
	void (*SendData)(uint8_t data, uint32_t len);
	void (*ReadData)(uint8_t * data, uint32_t readLen, uint32_t * outLen);
}USART_HAL_t;



E_USART_HAL_ERROR USART_HAL_SendData(const USART_HAL_t * const usart, uint8_t *data, uint32_t len);
E_USART_HAL_ERROR USART_HAL_ReadData(const USART_HAL_t * const usart, uint8_t *data, uint32_t readLen, uint32_t * outLen);



#endif /*__USART_HAL_H_*/
