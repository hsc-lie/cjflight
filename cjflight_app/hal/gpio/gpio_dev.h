#ifndef __GPIO_DEV_H_
#define __GPIO_DEV_H_

#include "common.h"


typedef enum
{
    GPIO_TYPE_LED0,
    GPIO_TYPE_LED1,
    GPIO_TYPE_LED2,

    GPIO_TYPE_MAX,
}GPIO_TYPE_t;


typedef struct
{
    void (*Init)(void);
    void (*DeInit)(void);
    void (*WritePinOut)(GPIO_TYPE_t type, uint8_t value);
    uint8_t (*ReadPinIn)(GPIO_TYPE_t type);
}GPIODev_t;

extern void GPIODevRegister(GPIODev_t *gpio);
extern void GPIODevUnregister(void);
extern void GPIODevInit(void);
extern void GPIODevDeInit(void);
extern void GPIODevWritePinOut(GPIO_TYPE_t type, uint8_t value);
extern uint8_t GPIODevReadPinIn(GPIO_TYPE_t type);

#endif /*__GPIO_DEV_H_*/