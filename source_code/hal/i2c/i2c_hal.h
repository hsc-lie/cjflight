#ifndef __I2C_HAL_H_
#define __I2C_HAL_H_

#include "common.h"


typedef enum
{
	E_I2C_ERROR_OK = 0,
	E_I2C_ERROR_NULL,
	E_I2C_ERROR_TIMEOUT,
}E_I2C_ERROR_T;



typedef struct
{
	void (*Init)();
	int (*SendData)(uint8_t, uint8_t *, uint32_t, uint8_t);
	int (*ReadData)(uint8_t, uint8_t *, uint32_t);
}I2C_HAL_t;


extern void I2C_HalInit(I2C_HAL_t * i2c);
extern int I2C_HalSendData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * data, uint32_t len, uint8_t isSendStop);
extern int I2C_HalReadData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * data, uint32_t len);
extern int I2C_HalReadRegData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen);




#endif

