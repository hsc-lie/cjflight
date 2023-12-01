#ifndef __I2C_HAL_H_
#define __I2C_HAL_H_

#include "common.h"


typedef enum
{
	E_I2C_ERROR_OK = 0,
	E_I2C_ERROR_NULL,
	E_I2C_ERROR_TIMEOUT,
}E_I2C_ERROR;



typedef struct
{
	void (*Init)();

	int (*SendData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
	int (*ReadData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
}I2C_HAL_t;


extern E_I2C_ERROR I2C_HalInit(I2C_HAL_t * i2c);
extern E_I2C_ERROR I2C_HalSendData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen);
extern E_I2C_ERROR I2C_HalReadData(I2C_HAL_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen);




#endif

