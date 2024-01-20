#include "mpu6050.h"

#define SAMPLE_RATE_DIVIDER               0x19        //陀螺仪采样频率寄存器地址
#define CONFIGURATION                     0x1a        //低通滤波频率
#define GYROSCOPE_CONFIGURATION           0x1b        //陀螺仪量程  0<<3 250度/s  1<<3 500度/s 2<<3 1000度/s 3<<3 2000度/s
#define ACCELEROMETER_CONFIGURATION       0x1c        //加速度量程 0<<3 2g   1<<3 4g   2<<3 8g   3<<3 16g
#define	ACCEL_XOUT_H	                  0x3B
#define	ACCEL_XOUT_L	                  0x3C
#define	ACCEL_YOUT_H	                  0x3D
#define	ACCEL_YOUT_L	                  0x3E
#define	ACCEL_ZOUT_H	                  0x3F
#define	ACCEL_ZOUT_L	                  0x40
#define TEMP_OUT_H                        0x41
#define TEMP_OUT_L                        0x42
#define	GYRO_XOUT_H				          0x43
#define	GYRO_XOUT_L				          0x44	
#define	GYRO_YOUT_H				          0x45
#define	GYRO_YOUT_L				          0x46
#define	GYRO_ZOUT_H				          0x47
#define	GYRO_ZOUT_L				          0x48
#define POWER_MANAGEMENT1                 0x6b         //电源管理1
#define WHO_AM_I                          0x75         //设备ID


/*
 * @函数名  MPU6050Init
 * @用  途  MPU6050初始化
 * @参  数  mpu6050:mpu6050结构体指针
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050Init(MPU6050_t *mpu6050)
{
	uint8_t writeData;
	uint8_t readData = 0xff;

	if((NULL == mpu6050)
		|| (NULL == mpu6050->I2CReadReg)
		|| (NULL == mpu6050->I2CWriteReg)
	)
	{
		return MPU6050_ERROR_t_NULL;
	}
	
	//检测陀螺仪	
	do
	{
		mpu6050->I2CReadReg(mpu6050->DevAddr, WHO_AM_I, &readData, 1);
	} while (readData != 0x98);
	
		
	writeData = 0x00;
	mpu6050->I2CWriteReg(mpu6050->DevAddr, POWER_MANAGEMENT1, &writeData, 1);      //MPU6050电源管理

	writeData = 0x01;
	mpu6050->I2CWriteReg(mpu6050->DevAddr, SAMPLE_RATE_DIVIDER, &writeData, 1);   //陀螺仪采样频率

	writeData = 0x02;
	mpu6050->I2CWriteReg(mpu6050->DevAddr, CONFIGURATION, &writeData, 1);         //低通滤波 
		
	//陀螺仪量程  0<<3 250度/s  1<<3 500度/s 2<<3 1000度/s 3<<3 2000度/s
	writeData = mpu6050->GyroRange << 3;
	mpu6050->I2CWriteReg(mpu6050->DevAddr,GYROSCOPE_CONFIGURATION, &writeData, 1);
	
	//加速度量程 0<<3 2g   1<<3 4g   2<<3 8g   3<<3 16g
	writeData = mpu6050->AccRange << 3;
	mpu6050->I2CWriteReg(mpu6050->DevAddr,ACCELEROMETER_CONFIGURATION, &writeData, 1);
	
	return MPU6050_ERROR_t_OK;
}

/*
 * @函数名  MPU6050GetBaseAcc
 * @用  途  获取MPU6050三轴加速度原始值
 * @参  数  mpu6050:mpu6050结构体指针
 *          acc:输出的三轴加速度原始值
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050GetBaseAcc(MPU6050_t *mpu6050, MPU6050BaseData_t *acc)
{
	uint8_t data[6] = {0};

	if(NULL == mpu6050->I2CReadReg)
	{
		return MPU6050_ERROR_t_NULL;
	}

	mpu6050->I2CReadReg(mpu6050->DevAddr, ACCEL_XOUT_H, data, 6);

	acc->X = (int16_t)((uint16_t)(data[0] << 8) | data[1]);
	acc->Y = (int16_t)((uint16_t)(data[2] << 8) | data[3]);
	acc->Z = (int16_t)((uint16_t)(data[4] << 8) | data[5]);

	return MPU6050_ERROR_t_OK;
}

/*
 * @函数名  MPU6050GetBaseGyro
 * @用  途  获取MPU6050三轴角速度原始值
 * @参  数  mpu6050:mpu6050结构体指针
 *          gyro:输出的三轴角速度原始值
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050GetBaseGyro(MPU6050_t *mpu6050, MPU6050BaseData_t *gyro)
{
	uint8_t data[6] = {0};

	if(NULL == mpu6050->I2CReadReg)
	{
		return MPU6050_ERROR_t_NULL;
	}

	mpu6050->I2CReadReg(mpu6050->DevAddr, GYRO_XOUT_H, data, 6);

	gyro->X = (int16_t)((uint16_t)(data[0] << 8) | data[1]) - mpu6050->GyroZero.X;
	gyro->Y = (int16_t)((uint16_t)(data[2] << 8) | data[3]) - mpu6050->GyroZero.Y;
	gyro->Z = (int16_t)((uint16_t)(data[4] << 8) | data[5]) - mpu6050->GyroZero.Z;

	return MPU6050_ERROR_t_OK;
}

/*
 * @函数名  MPU6050GetBaseAll
 * @用  途  获取MPU6050三轴加速度和三轴角速度原始值
 * @参  数  mpu6050:mpu6050结构体指针
 *          acc:输出的三轴加速度原始值
 *          gyro:输出的三轴角速度原始值
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050GetBaseAll(MPU6050_t *mpu6050, MPU6050BaseData_t *acc, MPU6050BaseData_t *gyro)
{
	uint8_t data[14] = {0};

	if(NULL == mpu6050->I2CReadReg)
	{
		return MPU6050_ERROR_t_NULL;
	}

	mpu6050->I2CReadReg(mpu6050->DevAddr, ACCEL_XOUT_H, data, 14);

	acc->X = (int16_t)((uint16_t)(data[0] << 8) | data[1]);
	acc->Y = (int16_t)((uint16_t)(data[2] << 8) | data[3]);
	acc->Z = (int16_t)((uint16_t)(data[4] << 8) | data[5]);

	gyro->X = (int16_t)((uint16_t)(data[8] << 8) | data[9]) - mpu6050->GyroZero.X;
	gyro->Y = (int16_t)((uint16_t)(data[10] << 8) | data[11]) - mpu6050->GyroZero.Y;
	gyro->Z = (int16_t)((uint16_t)(data[12] << 8) | data[13]) - mpu6050->GyroZero.Z;

	return MPU6050_ERROR_t_OK;
}

/*
 * @函数名  MPU6050GetBaseAcc
 * @用  途  MPU6050角速度原始值转实际值(单位度/s)
 * @参  数  mpu6050:mpu6050结构体指针
 *          in:输入的三轴角速度原始值
 *          out:输出的三轴角速度转换值(单位度/s)
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050ConvertDataGyro(MPU6050_t *mpu6050, MPU6050BaseData_t *in, MPU6050ConvertData_t *out)
{
	float gyroConvertBase;

	gyroConvertBase = (float)32768.0f / (float)(250u << mpu6050->GyroRange);

	out->X = (float)in->X/gyroConvertBase;
	out->Y = (float)in->Y/gyroConvertBase;
	out->Z = (float)in->Z/gyroConvertBase;

	return MPU6050_ERROR_t_OK;
}

/*
 * @函数名  MPU6050GetBaseAcc
 * @用  途  MPU6050加速度原始值转实际值(单位g)
 * @参  数  mpu6050:mpu6050结构体指针
 *          in:输入的三轴加速度原始值
 *          out:输出的三轴加速度转换值(单位g)
 * @返回值  错误的状态
*/
MPU6050_ERROR_t MPU6050ConvertDataAcc(MPU6050_t *mpu6050, MPU6050BaseData_t *in, MPU6050ConvertData_t *out)
{
	float accConvertBase;

	accConvertBase = (float)(32768u >> (mpu6050->AccRange + 1));
	//accConvertBase /= 9.8;

	out->X = (float)in->X/accConvertBase;
	out->Y = (float)in->Y/accConvertBase;
	out->Z = (float)in->Z/accConvertBase;

	return MPU6050_ERROR_t_OK;
}
