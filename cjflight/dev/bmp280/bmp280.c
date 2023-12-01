#include "bmp280.h"
//#include "math.h"


/****************************************************************************************
本文件代码参考自betaFlight
github网址 https://github.com/betaflight/betaflight
****************************************************************************************/



#define BMP280_DEFAULT_CHIP_ID			(0x58)

#define BMP280_CHIP_ID					(0xD0)  /* Chip ID Register */
#define BMP280_RST_REG					(0xE0)  /* Softreset Register */
#define BMP280_STAT_REG					(0xF3)  /* Status Register */
#define BMP280_CTRL_MEAS_REG			(0xF4)  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG				(0xF5)  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG			(0xF7)  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG			(0xF8)  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG		(0xF9)  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG		(0xFA)  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG		(0xFB)  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG		(0xFC)  /* Temperature XLSB Reg */


#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)

#define BMP280_OVERSAMP_SKIPPED         (0x00)
#define BMP280_OVERSAMP_1X              (0x01)
#define BMP280_OVERSAMP_2X              (0x02)
#define BMP280_OVERSAMP_4X              (0x03)
#define BMP280_OVERSAMP_8X              (0x04)
#define BMP280_OVERSAMP_16X             (0x05)

#define BMP280_FILTER_COEFF_OFF         (0x00)
#define BMP280_FILTER_COEFF_2           (0x01)
#define BMP280_FILTER_COEFF_4           (0x02)
#define BMP280_FILTER_COEFF_8           (0x03)
#define BMP280_FILTER_COEFF_16          (0x04)

#define BMP280_FORCED_MODE             	(0x01)
#define BMP280_NORMAL_MODE				(0x03)




//配置bmp气压计 气压和温度过采样
#define BMP280_PRESSURE_OSR         (BMP280_OVERSAMP_4X)
#define BMP280_TEMPERATURE_OSR      (BMP280_OVERSAMP_1X)
#define BMP280_MODE                 (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_FORCED_MODE)


//配置bmp280  iir滤波器
//#define BMP280_FILTER               (4 << 2)
#define BMP280_FILTER               (4 << 2)



static void BMP280_Delay(void)
{
	uint32_t i = 10000000;
	while(--i);
}



E_BMP280_ERROR BMP280_Init(BMP280_t * bmp280)
{
	uint8_t count = 0;
	uint8_t readData = 0;
	uint8_t writeData = 0;

	if((NULL == bmp280)
		|| (NULL == bmp280->I2CWriteReg)
		|| (NULL == bmp280->I2CReadReg)
	)
	{
		return E_BMP280_ERROR_NULL;
	}

	writeData = 0xb6;
	bmp280->I2CWriteReg(bmp280->DevAddr, BMP280_RST_REG, &writeData, 1);


	do
	{
		BMP280_Delay();
		bmp280->I2CReadReg(bmp280->DevAddr, BMP280_CHIP_ID, &readData, 1);
	}
	while(readData != BMP280_DEFAULT_CHIP_ID);
	

	bmp280->I2CReadReg(bmp280->DevAddr, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&bmp280->Calib, 24);

	writeData = BMP280_MODE;
	bmp280->I2CWriteReg(bmp280->DevAddr, BMP280_CTRL_MEAS_REG, &writeData, 1);

	writeData = BMP280_FILTER;
	bmp280->I2CWriteReg(bmp280->DevAddr, BMP280_CONFIG_REG, &writeData, 1);

	return  E_BMP280_ERROR_OK;
}



static uint32_t BMP280_CompensateTemp(BMP280_t * bmp280, int32_t temp)
{
    int32_t var1, var2, T;

    var1 = ((((temp >> 3) - ((int32_t)bmp280->Calib.T1 << 1))) * ((int32_t)bmp280->Calib.T2)) >> 11;
    var2  = (((((temp >> 4) - ((int32_t)bmp280->Calib.T1)) * ((temp >> 4) - ((int32_t)bmp280->Calib.T1))) >> 12) * ((int32_t)bmp280->Calib.T3)) >> 14;
    bmp280->Calib.TFine = var1 + var2;
    T = (bmp280->Calib.TFine * 5 + 128) >> 8;

    return T;
}


static uint32_t BMP280_CompensatePressure(BMP280_t * bmp280, int32_t pressure_value)
{
    int64_t var1, var2, p;
	
    var1 = ((int64_t)bmp280->Calib.TFine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280->Calib.P6;
    var2 = var2 + ((var1*(int64_t)bmp280->Calib.P5) << 17);
    var2 = var2 + (((int64_t)bmp280->Calib.P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280->Calib.P3) >> 8) + ((var1 * (int64_t)bmp280->Calib.P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280->Calib.P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - pressure_value;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280->Calib.P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280->Calib.P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280->Calib.P7) << 4);
	
    return (uint32_t)p;
}







E_BMP280_ERROR BMP280_Startup(BMP280_t * bmp280)
{
	uint8_t writeData = BMP280_MODE;

	if((NULL == bmp280)
		|| (NULL == bmp280->I2CWriteReg)
	)
	{
		return E_BMP280_ERROR_NULL;
	}

	bmp280->I2CWriteReg(bmp280->DevAddr, BMP280_CTRL_MEAS_REG, &writeData, 1);

	return  E_BMP280_ERROR_OK;
}


E_BMP280_ERROR BMP280_GetData(BMP280_t * bmp280, BMP280_Data_t * data)
{
    uint8_t dataBuffer[6] = {0};

    int32_t pressure = 0;
    int32_t temperature = 0;


	if((NULL == bmp280)
		|| (NULL == bmp280->I2CReadReg)
	)
	{
		return E_BMP280_ERROR_NULL;
	}

	bmp280->I2CReadReg(bmp280->DevAddr, BMP280_PRESSURE_MSB_REG, dataBuffer, 6);

	pressure = (int32_t)((((uint32_t)(dataBuffer[0])) << 12) | (((uint32_t)(dataBuffer[1])) << 4) | ((uint32_t)dataBuffer[2] >> 4));
	temperature = (int32_t)((((uint32_t)(dataBuffer[3])) << 12) | (((uint32_t)(dataBuffer[4])) << 4) | ((uint32_t)dataBuffer[5] >> 4));

    data->Temp = (float)BMP280_CompensateTemp(bmp280, temperature)/100.0f;	
	data->Pressure = (float)BMP280_CompensatePressure(bmp280, pressure)/256.0f;	

	return  E_BMP280_ERROR_OK;
}







