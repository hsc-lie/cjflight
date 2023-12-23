#ifndef __I2C_DEV_H_
#define __I2C_DEV_H_

#include "common.h"
#include "dev.h"




typedef enum
{
	I2C_TYPE_SENSOR,

	I2C_MAX
}I2C_t;


typedef enum
{
	I2C_DEV_ERROR_OK = 0,
	I2C_DEV_ERROR_NULL,
	I2C_DEV_ERROR_INVALID,
	I2C_DEV_ERROR_TIMEOUT,
}I2C_DEV_ERROR_t;


typedef struct
{
	I2C_DEV_ERROR_t (*Init)(void);
	I2C_DEV_ERROR_t (*DeInit)(void);
	I2C_DEV_ERROR_t (*SendData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
	I2C_DEV_ERROR_t (*ReadData)(uint8_t addr, uint8_t * reg, uint32_t regLen, uint8_t * data, uint32_t dataLen);
}I2CDev_t;

extern I2C_DEV_ERROR_t I2CDevRegister(I2C_t i2c, I2CDev_t *dev);
extern I2C_DEV_ERROR_t I2CDevUnregister(I2C_t i2c);
extern I2C_DEV_ERROR_t I2CDevInit(I2C_t i2c);
extern I2C_DEV_ERROR_t I2CDevDeInit(I2C_t i2c);
extern I2C_DEV_ERROR_t I2CDevSendData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen);
extern I2C_DEV_ERROR_t I2CDevReadData(I2C_t i2c, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen);

#endif

