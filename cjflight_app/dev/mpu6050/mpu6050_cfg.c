#include "mpu6050_cfg.h"

#include "i2c_dev.h"

static void MPU6050I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevSendData(I2C_DEV_SENSOR, addr, &reg, 1, data, len);
}

static void MPU6050I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	I2CDevReadData(I2C_DEV_SENSOR, addr, &reg, 1, data, len);
}

MPU6050_t MPU6050 = 
{
	.DevAddr = MPU6050_DEV_ADDR1,
	.GyroRange = MPU6050_GYRO_RANGE_2000,
	.AccRange = MPU6050_ACC_RANGE_8G,
	.GyroZero = 
	{
		.X = -7,
		.Y = 54,
		.Z = -1,
	},
	.I2CWriteReg = MPU6050I2CWriteReg,
	.I2CReadReg = MPU6050I2CReadReg,
};
