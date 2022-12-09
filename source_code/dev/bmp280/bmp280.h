#ifndef _BMP280_H_
#define _BMP280_H_


#include "common.h"


/****************************************************************************************
本文件代码参考自betaFlight
github网址 https://github.com/betaflight/betaflight
****************************************************************************************/




#define BMP280_ID					(0x76)


/*
typedef struct 
{
    float pressure;
    float temp;
    float altitude;
}bmp280_data_t;
*/


typedef enum
{
	E_BMP280_ERROR_OK,
	E_BMP280_ERROR_NULL,
	E_BMP280_ERROR_NOT_FIND_DEV,
}E_BMP280_ERROR;







typedef struct
{
	uint8_t DevAddr;

	void (*I2CWriteReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
	void (*I2CReadReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

	struct 
	{
	    uint16_t T1;	/* calibration T1 data */
	    int16_t T2; /* calibration T2 data */
	    int16_t T3; /* calibration T3 data */
	    uint16_t P1;	/* calibration P1 data */
	    int16_t P2; /* calibration P2 data */
	    int16_t P3; /* calibration P3 data */
	    int16_t P4; /* calibration P4 data */
	    int16_t P5; /* calibration P5 data */
	    int16_t P6; /* calibration P6 data */
	    int16_t P7; /* calibration P7 data */
	    int16_t P8; /* calibration P8 data */
	    int16_t P9; /* calibration P9 data */
	    int32_t TFine; /* calibration t_fine data */
	} Calib;
}BMP280_t;


typedef struct
{
	float Pressure;
    float Temp;
    float Altitude;
}BMP280_Data_t;



extern E_BMP280_ERROR BMP280_Init(BMP280_t * bmp280);
extern E_BMP280_ERROR BMP280_Startup(BMP280_t * bmp280);
extern E_BMP280_ERROR BMP280_GetData(BMP280_t * bmp280, BMP280_Data_t * data);


#endif /*_BMP180_H_*/
