#ifndef __MPU6050_H
#define __MPU6050_H

#include "common.h"

#define MPU6050_DEV_ADDR1                0x68
#define MPU6050_DEV_ADDR2                0x69   

typedef enum
{
	MPU6050_ERROR_t_OK,
	MPU6050_ERROR_t_NULL,
	MPU6050_ERROR_t_TIMEOUT,
	MPU6050_ERROR_t_NOT_FIND_DEV,
}MPU6050_ERROR_t;


//单位 度/s
typedef enum
{
	MPU6050_GYRO_RANGE_250 = 0,
	MPU6050_GYRO_RANGE_500,
	MPU6050_GYRO_RANGE_1000,
	MPU6050_GYRO_RANGE_2000,
}MPU6050_GYRO_RANGE_t;

//单位 g
typedef enum
{
	MPU6050_ACC_RANGE_2G = 0,
	MPU6050_ACC_RANGE_4G,
	MPU6050_ACC_RANGE_8G,
	MPU6050_ACC_RANGE_16G,
}MPU6050_ACC_RANGE_t;

typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
	
}MPU6050BaseData_t;

typedef struct
{
	float X;
	float Y;
	float Z;
}MPU6050ConvertData_t;

typedef struct
{
	uint8_t DevAddr;

	MPU6050_GYRO_RANGE_t GyroRange;
	MPU6050_ACC_RANGE_t AccRange;
	
	void (*I2CWriteReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
	void (*I2CReadReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

	MPU6050BaseData_t GyroZero;
}MPU6050_t;

extern MPU6050_ERROR_t MPU6050Init(MPU6050_t * mpu6050);
extern MPU6050_ERROR_t MPU6050GetBaseGyro(MPU6050_t * mpu6050, MPU6050BaseData_t * gyro);
extern MPU6050_ERROR_t MPU6050GetBaseAcc(MPU6050_t * mpu6050, MPU6050BaseData_t * acc);
extern MPU6050_ERROR_t MPU6050GetBaseAll(MPU6050_t *mpu6050, MPU6050BaseData_t *acc, MPU6050BaseData_t *gyro);
extern MPU6050_ERROR_t MPU6050ConvertDataGyro(MPU6050_t * mpu6050, MPU6050BaseData_t * in, MPU6050ConvertData_t * out);
extern MPU6050_ERROR_t MPU6050ConvertDataAcc(MPU6050_t * mpu6050, MPU6050BaseData_t * in, MPU6050ConvertData_t * out);

#endif /*__MPU6050_H_*/
