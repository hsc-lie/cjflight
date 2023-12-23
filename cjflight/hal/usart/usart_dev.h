#ifndef __USART_DEV_H_
#define __USART_DEV_H_

#include "common.h"
#include "dev.h"

typedef enum
{
	USART_DEV_ERROR_OK,
	USART_DEV_ERROR_NULL,
}USART_DEV_ERROR_t;


typedef struct
{
	Dev_t Dev;

	void (*SendData)(uint8_t * data, uint32_t len);
	void (*ReadData)(uint8_t * data, uint32_t readLen, uint32_t * outLen);
}USARTDev_t;


extern void USARTDevInit(USARTDev_t * i2c);
extern void USARTDevDeInit(USARTDev_t * i2c);
extern void USARTDevInitAll();
extern void USARTDevDeInitAll();
extern USARTDev_t *USARTDevGet(uint32_t id);
extern void USARTDevRegister(USARTDev_t *USARTDev);
extern void USARTDevUnregister(USARTDev_t *USARTDev);

USART_DEV_ERROR_t USARTDevSendData(const USARTDev_t * const usart, uint8_t *data, uint32_t len);
USART_DEV_ERROR_t USARTDevReadData(const USARTDev_t * const usart, uint8_t *data, uint32_t readLen, uint32_t * outLen);



#endif /*__USART_DEV_H_*/
