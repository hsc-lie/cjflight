#ifndef __SPL06_H_
#define __SPL06_H_

#include "stdio.h"
#include "common.h"


#define SPL06_ADDR1              0x76
#define SPL06_ADDR2              0x77


typedef enum
{
	E_SPL06_ERROR_OK = 0,
	E_SPL06_ERROR_NULL,
	E_SPL06_ERROR_DEV_NOT_FOUND,

}E_SPL06_ERROR;


typedef enum
{
	E_SPL06_RATE_1 = 0,
	E_SPL06_RATE_2,
	E_SPL06_RATE_4,
	E_SPL06_RATE_8,
	E_SPL06_RATE_16,
	E_SPL06_RATE_32,
	E_SPL06_RATE_64,
	E_SPL06_RATE_128,
}E_SPL06_RATE;

typedef enum
{
	E_SPL06_PRC_TIMES1 = 0,
	E_SPL06_PRC_TIMES2,
	E_SPL06_PRC_TIMES4,
	E_SPL06_PRC_TIMES8,
	E_SPL06_PRC_TIMES16,
	E_SPL06_PRC_TIMES32,
	E_SPL06_PRC_TIMES64,
	E_SPL06_PRC_TIMES128,
}E_SPL06_PRC;



typedef enum
{
	E_SPL06_MODE_STOP = 0,                      //停止测量
	
	E_SPL06_MODE_PRESSURE_MEASUREMENT = 1,          //气压单次测量
	E_SPL06_MODE_TEMPERATURE_MEASUREMENT = 2,       //温度单次测量
	
	E_SPL06_MODE_CONTINUOUS_PRESSURE_MEASUREMENT = 5,    //气压连续测量
	E_SPL06_MODE_CONTINUOUS_TEMPERATURE_MEASUREMENT = 6,    //温度连续测量
	E_SPL06_MODE_CONTINUOUS_ALL_MEASUREMENT = 7,    //气压和温度连续测量
	
}E_SPL06_MODE;



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
}SPL06_CalibParams_t;



typedef struct
{
	uint8_t DevAddr;
	
	E_SPL06_RATE PressureRate;
	E_SPL06_PRC  PressurePRC;
	E_SPL06_RATE TemperatureRate;
	E_SPL06_PRC  TemperaturePRC;

	E_SPL06_MODE Mode;

	void (*I2CWriteReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
	void (*I2CReadReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

	/*********以下成员不需要用户去配置***********/
	SPL06_CalibParams_t CalibParams;

	int32_t RawPressure;
    int32_t RawTemperature;
    int32_t KP;    
    int32_t KT;

}SPL06_t;

	



typedef struct
{
    float Pressure;
    float Temperature;
}SPL06_Data_t;


extern E_SPL06_ERROR SPL06_Init(SPL06_t * spl06);

extern void SPL06_SetPressureRate(SPL06_t * spl06, E_SPL06_RATE rate, E_SPL06_PRC prc);
extern void SPL06_SetTemperatureRate(SPL06_t * spl06, E_SPL06_RATE rate, E_SPL06_PRC prc);
extern void SPL06_SetMode(SPL06_t * spl06, E_SPL06_MODE mode);

extern float SPL06_GetPressure(SPL06_t * spl06);
extern float SPL06_GetTemperature(SPL06_t * spl06);
extern void SPL06_GetDataAll(SPL06_t * spl06, SPL06_Data_t * data);




#endif /*__SPL06_H_*/
