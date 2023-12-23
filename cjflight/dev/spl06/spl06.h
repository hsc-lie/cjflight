#ifndef __SPL06_H_
#define __SPL06_H_

//#include "stdio.h"
#include "common.h"


#define SPL06_ADDR1              0x76
#define SPL06_ADDR2              0x77


typedef enum
{
	SPL06_ERROR_OK = 0,
	SPL06_ERROR_NULL,
	SPL06_ERROR_DEV_NOT_FOUND,

}SPL06_ERROR_t;


typedef enum
{
	SPL06_RATE_1 = 0,
	SPL06_RATE_2,
	SPL06_RATE_4,
	SPL06_RATE_8,
	SPL06_RATE_16,
	SPL06_RATE_32,
	SPL06_RATE_64,
	SPL06_RATE_128,
}SPL06_RATE_t;

typedef enum
{
	SPL06_PRC_TIMES1 = 0,
	SPL06_PRC_TIMES2,
	SPL06_PRC_TIMES4,
	SPL06_PRC_TIMES8,
	SPL06_PRC_TIMES16,
	SPL06_PRC_TIMES32,
	SPL06_PRC_TIMES64,
	SPL06_PRC_TIMES128,
}SPL06_PRC_t;



typedef enum
{
	SPL06_MODE_STOP = 0,                      //停止测量

	SPL06_MODE_PRESSURE_MEASUREMENT = 1,          //气压单次测量
	SPL06_MODE_TEMPERATURE_MEASUREMENT = 2,       //温度单次测量

	SPL06_MODE_CONTINUOUS_PRESSURE_MEASUREMENT = 5,    //气压连续测量
	SPL06_MODE_CONTINUOUS_TEMPERATURE_MEASUREMENT = 6,    //温度连续测量
	SPL06_MODE_CONTINUOUS_ALL_MEASUREMENT = 7,    //气压和温度连续测量
	
}SPL06_MODE_t;



typedef struct  
{	
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
}SPL06CalibParams_t;



typedef struct
{
	uint8_t DevAddr;
	
	SPL06_RATE_t PressureRate;
	SPL06_PRC_t  PressurePRC;
	SPL06_RATE_t TemperatureRate;
	SPL06_PRC_t  TemperaturePRC;

	SPL06_MODE_t Mode;

	void (*I2CWriteReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
	void (*I2CReadReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

	/*********以下成员不需要用户去配置***********/
	SPL06CalibParams_t CalibParams;

	int32_t RawPressure;
    int32_t RawTemperature;
    int32_t KP;    
    int32_t KT;

}SPL06_t;

	



typedef struct
{
    float Pressure;
    float Temperature;
}SPL06Data_t;


extern SPL06_ERROR_t SPL06_Init(SPL06_t * spl06);

extern void SPL06SetPressureRate(SPL06_t * spl06, SPL06_RATE_t rate, SPL06_PRC_t prc);
extern void SPL06SetTemperatureRate(SPL06_t * spl06, SPL06_RATE_t rate, SPL06_PRC_t prc);
extern void SPL06SetMode(SPL06_t * spl06, SPL06_MODE_t mode);

extern float SPL06GetPressure(SPL06_t * spl06);
extern float SPL06GetTemperature(SPL06_t * spl06);
extern void SPL06GetDataAll(SPL06_t * spl06, SPL06Data_t * data);




#endif /*__SPL06_H_*/
