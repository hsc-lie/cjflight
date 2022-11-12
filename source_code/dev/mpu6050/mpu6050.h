#ifndef __MPU6050_H
#define __MPU6050_H


#include "common.h"

#define MPU6050_DEV_ADDR1                0x68
#define MPU6050_DEV_ADDR2                0x69   



typedef enum
{
	E_MPU6050_ERROR_OK,
	E_MPU6050_ERROR_NULL,
	E_MPU6050_ERROR_NOT_FIND_DEV,
}E_MPU6050_ERROR;


//单位 度/s
typedef enum
{
	E_MPU6050_GYRO_RANGE_250 = 0,
	E_MPU6050_GYRO_RANGE_500,
	E_MPU6050_GYRO_RANGE_1000,
	E_MPU6050_GYRO_RANGE_2000,
}E_MPU6050_GYRO_RANGE;

//单位 g
typedef enum
{
	E_MPU6050_ACC_RANGE_2G = 0,
	E_MPU6050_ACC_RANGE_4G,
	E_MPU6050_ACC_RANGE_8G,
	E_MPU6050_ACC_RANGE_16G,
}E_MPU6050_ACC_RANGE;


typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
	
}MPU6050_BaseData_t;

typedef struct
{
	float X;
	float Y;
	float Z;
}MPU6050_ConvertData_t;



typedef struct
{
	uint8_t DevAddr;

	E_MPU6050_GYRO_RANGE GyroRange;
	E_MPU6050_ACC_RANGE AccRange;
	
	void (*I2CSendData)(uint8_t addr, uint8_t *data, uint32_t len, uint8_t isSendStop);
	void (*I2CReadData)(uint8_t addr, uint8_t *data, uint32_t len);

	MPU6050_BaseData_t GyroZero;
}MPU6050_t;



void i2c_config(void);

void mpu6050_init(void);
void mpu6050_biquad_fiter_parameter_init(void);

uint8_t mpu6050_get_zero(void);
void gyro_data_change(void);
void mpu6050_get_gyro(Mpu6050_Data_t * Gyro);
void mpu6050_get_acc(Mpu6050_Data_t * Acc);

void biquad_fiter_parameter_init(void);


#endif /*__MPU6050_H_*/
