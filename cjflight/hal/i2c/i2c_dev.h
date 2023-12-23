#ifndef __I2C_DEV_H_
#define __I2C_DEV_H_

#include "common.h"
#include "dev.h"

typedef enum
{
	I2C_ERROR_OK = 0,
	I2C_ERROR_NULL,
	I2C_ERROR_TIMEOUT,
}I2C_ERROR_t;



typedef struct
{
	Dev_t Dev;

	I2C_ERROR_t (*SendData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
	I2C_ERROR_t (*ReadData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
}I2CDev_t;


extern void I2CDevInit(I2CDev_t * i2c);
extern void I2CDevDeInit(I2CDev_t * i2c);
extern void I2CDevInitAll();
extern void I2CDevDeInitAll();
extern I2CDev_t *I2CDevGet(uint32_t id);
extern I2C_ERROR_t I2CDevSendData(I2CDev_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen);
extern I2C_ERROR_t I2CDevReadData(I2CDev_t * i2c, uint8_t addr, uint8_t * reg, uint32_t regLen,uint8_t * data, uint32_t dataLen);

#endif

