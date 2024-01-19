#ifndef __SIMULATION_I2C_H_
#define __SIMULATION_I2C_H_

#include "common.h"

typedef enum
{
	SIMULATION_I2C_ERROR_OK,
	SIMULATION_I2C_ERROR_NULL,
	SIMULATION_I2C_ERROR_NACK,
}SIMULATION_I2C_ERROR_t;

typedef struct
{
	uint32_t DelayCount;                                     //软件延时的计数
	void (*SCLSet0)(void);                                   //设置SCL引脚低电平的函数
	void (*SCLSet1)(void);                                   //设置SCL引脚高电平的函数
	void (*SDASet0)(void);                                   //设置SDA引脚低电平的函数
	void (*SDASet1)(void);                                   //设置SDA引脚高电平的函数
	void (*SDASetTX)(void);                                  //设置SDA引脚方向为输入的函数
	void (*SDASetRX)(void);                                  //设置SDA引脚方向为输出的函数
	uint8_t (*SDARead)(void);                                //读取SDA引脚电平的函数
}SimulationI2C_t;

extern SIMULATION_I2C_ERROR_t SimulationI2CWriteData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint32_t regLen, uint8_t *data, uint8_t dataLen);
extern SIMULATION_I2C_ERROR_t SimulationI2CReadData(SimulationI2C_t *i2c, uint8_t addr, uint8_t *reg, uint8_t regLen, uint8_t *data, uint8_t dataLen);

#endif /*__SIMULATION_I2C_H_*/