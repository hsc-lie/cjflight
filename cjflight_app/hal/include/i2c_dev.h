#ifndef __I2C_DEV_H_
#define __I2C_DEV_H_

#include "dev.h"

typedef enum
{
	I2C_DEV_SENSOR,
	I2C_DEV_NUM_MAX
}I2C_DEV_NUM_t;

typedef enum
{
	I2C_DEV_OK = 0,                             //成功
	I2C_DEV_ERROR_INVALID_PARAM,                //无效的参数
	I2C_DEV_ERROR_UNREGISTERED,                 //设备未注册
	I2C_DEV_ERROR_NULL_FUNC,                    //空的函数
	I2C_DEV_ERROR_TIMEOUT,                      //超时
}I2C_DEV_ERROR_t;

typedef I2C_DEV_ERROR_t (*I2CDevInitFunc_t)(void);
typedef I2C_DEV_ERROR_t (*I2CDevDeInitFunc_t)(void);
typedef I2C_DEV_ERROR_t (*I2CDevSendDataFunc_t)(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen);
typedef I2C_DEV_ERROR_t (*I2CDevReadDataFunc_t)(uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint32_t dataLen);

typedef struct
{
	I2CDevInitFunc_t Init;
	I2CDevDeInitFunc_t DeInit;
	I2CDevSendDataFunc_t SendData;
	I2CDevReadDataFunc_t ReadData;
}I2CDev_t;

extern I2C_DEV_ERROR_t I2CDevRegister(I2C_DEV_NUM_t num, I2CDev_t *dev);
extern I2C_DEV_ERROR_t I2CDevUnregister(I2C_DEV_NUM_t num);
extern I2C_DEV_ERROR_t I2CDevInit(I2C_DEV_NUM_t num);
extern I2C_DEV_ERROR_t I2CDevDeInit(I2C_DEV_NUM_t num);
extern I2C_DEV_ERROR_t I2CDevSendData(I2C_DEV_NUM_t num, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen);
extern I2C_DEV_ERROR_t I2CDevReadData(I2C_DEV_NUM_t num, uint8_t addr, uint8_t *reg, uint32_t regLen,uint8_t *data, uint32_t dataLen);

#endif /*__I2C_DEV_H_*/

