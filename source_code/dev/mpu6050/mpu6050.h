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
	
	void (*I2CWriteReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
	void (*I2CReadReg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);

	MPU6050_BaseData_t GyroZero;
}MPU6050_t;

extern E_MPU6050_ERROR MPU6050_Init(MPU6050_t * mpu6050);
extern E_MPU6050_ERROR MPU6050_GetBaseGyro(MPU6050_t * mpu6050, MPU6050_BaseData_t * gyro);
extern E_MPU6050_ERROR MPU6050_GetBaseAcc(MPU6050_t * mpu6050, MPU6050_BaseData_t * acc);
extern E_MPU6050_ERROR MPU6050_ConvertDataGyro(MPU6050_t * mpu6050, MPU6050_BaseData_t * in, MPU6050_ConvertData_t * out);
extern E_MPU6050_ERROR MPU6050_ConvertDataAcc(MPU6050_t * mpu6050, MPU6050_BaseData_t * in, MPU6050_ConvertData_t * out);

#endif /*__MPU6050_H_*/
