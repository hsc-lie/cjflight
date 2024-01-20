#ifndef __USART_DEV_H_
#define __USART_DEV_H_

#include "dev.h"

typedef enum
{
	USART_DEV_PRINTF,
	USART_DEV_REMOTE,
	USART_DEV_NUM_MAX
}USART_DEV_NUM_t;

typedef enum
{
	USART_DEV_OK = 0,                             //成功
	USART_DEV_ERROR_INVALID_PARAM,                //无效的参数
	USART_DEV_ERROR_UNREGISTERED,                 //设备未注册
	USART_DEV_ERROR_NULL_FUNC,                    //空的函数
}USART_DEV_ERROR_t;


typedef struct
{
	USART_DEV_ERROR_t (*Init)(void);
	USART_DEV_ERROR_t (*DeInit)(void);
	USART_DEV_ERROR_t (*SendData)(uint8_t *data, uint32_t len);
	USART_DEV_ERROR_t (*ReadData)(uint8_t *data, uint32_t readLen, uint32_t *outLen);
}USARTDev_t;

extern USART_DEV_ERROR_t USARTDevRegister(USART_DEV_NUM_t num, USARTDev_t *USARTDev);
extern USART_DEV_ERROR_t USARTDevUnregister(USART_DEV_NUM_t num);
extern USART_DEV_ERROR_t USARTDevInit(USART_DEV_NUM_t num);
extern USART_DEV_ERROR_t USARTDevDeInit(USART_DEV_NUM_t num);
extern USART_DEV_ERROR_t USARTDevSendData(USART_DEV_NUM_t num, uint8_t *data, uint32_t len);
extern USART_DEV_ERROR_t USARTDevReadData(USART_DEV_NUM_t num, uint8_t *data, uint32_t readLen, uint32_t *outLen);

#endif /*__USART_DEV_H_*/
