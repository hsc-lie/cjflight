#include "mpu6050_cfg.h"

#include "i2c_dev.h"

static I2CDev_t *MPU6050I2CDev = NULL;

static void MPU6050I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	if(NULL == MPU6050I2CDev)
	{
		MPU6050I2CDev = I2CDevGet(0);
	}

	if (NULL != MPU6050I2CDev)
	{
		I2CDevSendData(MPU6050I2CDev, addr, &reg, 1, data, len);
	}
	
}

static void MPU6050I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	if(NULL == MPU6050I2CDev)
	{
		MPU6050I2CDev = I2CDevGet(0);
	}

	if (NULL != MPU6050I2CDev)
	{
		I2CDevReadData(MPU6050I2CDev, addr, &reg, 1, data, len);
	}
}


MPU6050_t MPU6050 = 
{
	.DevAddr = MPU6050_DEV_ADDR1,
	
	.GyroRange = MPU6050_GYRO_RANGE_2000,
	.AccRange = MPU6050_ACC_RANGE_2G,

	.GyroZero = 
	{
		.X = 0,
		.Y = 0,
		.Z = 0,
	},

	.I2CWriteReg = MPU6050I2CWriteReg,
	.I2CReadReg = MPU6050I2CReadReg,
};







