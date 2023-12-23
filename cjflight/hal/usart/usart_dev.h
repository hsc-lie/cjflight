#ifndef __USART_DEV_H_
#define __USART_DEV_H_

#include "common.h"
#include "dev.h"


typedef enum
{
	USART_PRINTF,
	USART_REMOTE,

	USART_MAX
}USART_t;


typedef enum
{
	USART_DEV_ERROR_OK,
	USART_DEV_ERROR_NULL,
	USART_DEV_ERROR_INVALID,
}USART_DEV_ERROR_t;


typedef struct
{
	USART_DEV_ERROR_t (*Init)(void);
	USART_DEV_ERROR_t (*DeInit)(void);
	USART_DEV_ERROR_t (*SendData)(uint8_t * data, uint32_t len);
	USART_DEV_ERROR_t (*ReadData)(uint8_t * data, uint32_t readLen, uint32_t * outLen);
}USARTDev_t;

extern USART_DEV_ERROR_t USARTDevRegister(USART_t usart, USARTDev_t *USARTDev);
extern USART_DEV_ERROR_t USARTDevUnregister(USART_t usart);
extern USART_DEV_ERROR_t USARTDevInit(USART_t usart);
extern USART_DEV_ERROR_t USARTDevDeInit(USART_t usart);
extern USART_DEV_ERROR_t USARTDevSendData(USART_t usart, uint8_t *data, uint32_t len);
extern USART_DEV_ERROR_t USARTDevReadData(USART_t usart, uint8_t *data, uint32_t readLen, uint32_t * outLen);



#endif /*__USART_DEV_H_*/
