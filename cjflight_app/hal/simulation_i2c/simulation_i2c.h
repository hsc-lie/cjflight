#ifndef __SIMULATION_I2C_H_
#define __SIMULATION_I2C_H_

#include "common.h"

typedef enum
{
	SIMULATION_I2C_ERROR_OK,
	SIMULATION_I2C_ERROR_NULL,
	SIMULATION_I2C_ERROR_NACK,
}SIMULATION_I2C_ERROR_t;

typedef enum
{
	SIMULATION_I2C_SDA_RX,
	SIMULATION_I2C_SDA_TX,
}SIMULATION_I2C_SDA_DIR_t;

typedef struct
{
	uint32_t DelayCount;                                     //软件延时的计数
	void (*SCLSet)(uint8_t value);                           //设置SCL引脚高低电平的函数
	void (*SDASet)(uint8_t value);                           //设置SDA引脚高低电平的函数
	void (*SDADirSet)(SIMULATION_I2C_SDA_DIR_t dir);         //设置SDA引脚方向(输入输出)的函数
	uint8_t (*SDARead)(void);                                //读取SDA引脚电平的函数
}SimulationI2C_t;

extern SIMULATION_I2C_ERROR_t SimulationI2CWriteData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint8_t dataLen);
extern SIMULATION_I2C_ERROR_t SimulationI2CReadData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint8_t regLen, uint8_t *data, uint8_t dataLen);

#endif /*__SIMULATION_I2C_H_*/