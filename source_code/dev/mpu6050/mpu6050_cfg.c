#include "mpu6050_cfg.h"

#include "i2c_hal_cfg.h"

static void MPU6050_I2CWriteReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalSendData(&I2C_Dev1, addr, &reg, 1, data, len);
}

static void MPU6050_I2CReadReg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
	
	I2C_HalReadData(&I2C_Dev1, addr, &reg, 1, data, len);
}


MPU6050_t MPU6050 = 
{
	.DevAddr = MPU6050_DEV_ADDR1,
	
	.GyroRange = E_MPU6050_GYRO_RANGE_2000,
	.AccRange = E_MPU6050_ACC_RANGE_2G,

	.GyroZero = 
	{
		.X = 0,
		.Y = 0,
		.Z = 0,
	},

	.I2CWriteReg = MPU6050_I2CWriteReg,
	.I2CReadReg = MPU6050_I2CReadReg,
};







