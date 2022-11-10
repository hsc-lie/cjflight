#include "bmp280.h"
#include "math.h"
#include "filter.h"

#include "filter.h"


//配置bmp气压计 气压和温度过采样
#define BMP280_PRESSURE_OSR         (BMP280_OVERSAMP_16X)
#define BMP280_TEMPERATURE_OSR      (BMP280_OVERSAMP_4X)
#define BMP280_MODE                 (BMP280_PRESSURE_OSR << 2 | BMP280_TEMPERATURE_OSR << 5 | BMP280_NORMAL_MODE)


//配置bmp280  iir滤波器
//#define BMP280_FILTER               (4 << 2)
#define BMP280_FILTER               (4 << 2)



bmp280_data_t bmp280_data = {0};

biquadFilter_t bmp180_biquad_parameter;


struct 
{
    uint16_t dig_T1;	/* calibration T1 data */
    int16_t dig_T2; /* calibration T2 data */
    int16_t dig_T3; /* calibration T3 data */
    uint16_t dig_P1;	/* calibration P1 data */
    int16_t dig_P2; /* calibration P2 data */
    int16_t dig_P3; /* calibration P3 data */
    int16_t dig_P4; /* calibration P4 data */
    int16_t dig_P5; /* calibration P5 data */
    int16_t dig_P6; /* calibration P6 data */
    int16_t dig_P7; /* calibration P7 data */
    int16_t dig_P8; /* calibration P8 data */
    int16_t dig_P9; /* calibration P9 data */
    int32_t t_fine; /* calibration t_fine data */
} bmp280_calib;



static void bmp280_delay(void)
{
	uint32_t i = 10000000;
	while(--i);
}



void bmp280_init()
{
    uint8_t id = 0u;
	bmp280_delay();
	
	simulation_i2c_writereg(BMP280_ID,BMP280_RST_REG,0xb6);
    while(id != BMP280_DEFAULT_CHIP_ID)
	{

		simulation_i2c_readregs(BMP280_ID, BMP280_CHIP_ID,1,&id);
        bmp280_delay();
	}
	
    simulation_i2c_readregs(BMP280_ID, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG,24,(uint8_t*)&bmp280_calib);

    simulation_i2c_writereg(BMP280_ID,BMP280_CTRL_MEAS_REG,BMP280_MODE);
    simulation_i2c_writereg(BMP280_ID,BMP280_CONFIG_REG,BMP280_FILTER);
	
	bmp280_delay();
	
}


static uint32_t bmp280_compensate_t(int32_t temp_value)
{
    int32_t var1, var2, T;

    var1 = ((((temp_value >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2  = (((((temp_value >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((temp_value >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    bmp280_calib.t_fine = var1 + var2;
    T = (bmp280_calib.t_fine * 5 + 128) >> 8;

    return T;
}


static uint32_t bmp280_compensate_p(int32_t pressure_value)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - pressure_value;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (uint32_t)p;
}

#define P
static float pressureToAltitude(const float pressure)
{
    
    //return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
    return 44300.0f * (1.0f - powf(pressure/101325.0f, 0.190257519));
}


void bmp280_data_get(bmp280_data_t * bmp280_data)
{
    uint8_t data[6] = {0};

    int32_t pressure = 0;
    int32_t temperature = 0;
	
    simulation_i2c_readregs(BMP280_ID, BMP280_PRESSURE_MSB_REG,6,data);

	pressure = (int32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
	temperature = (int32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));

    bmp280_data->temp = (float)bmp280_compensate_t(temperature)/100.0f;	
	bmp280_data->pressure = (float)bmp280_compensate_p(pressure)/256.0f;	

	bmp280_data->altitude = pressureToAltitude(bmp280_data->pressure) * 100.0f;

}







