#include "mpu6050.h"



#define SAMPLE_RATE_DIVIDER               0x19        //陀螺仪采样频率寄存器地址
#define CONFIGURATION                     0x1a        //低通滤波频率
#define GYROSCOPE_CONFIGURATION           0x1b        //陀螺仪量程  0<<3 250度/s  1<<3 500度/s 2<<3 1000度/s 3<<3 2000度/s
#define ACCELEROMETER_CONFIGURATION       0x1c        //加速度量程 0<<3 2g   1<<3 4g   2<<3 8g   3<<3 16g
#define	ACCEL_XOUT_H	        0x3B
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	GYRO_XOUT_H				0x43
#define	GYRO_XOUT_L				0x44	
#define	GYRO_YOUT_H				0x45
#define	GYRO_YOUT_L				0x46
#define	GYRO_ZOUT_H				0x47
#define	GYRO_ZOUT_L				0x48
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define POWER_MANAGEMENT1       0x6b         //电源管理1
#define WHO_AM_I                0x75         //我是谁





E_MPU6050_ERROR MPU6050_Init(MPU6050_t * mpu6050)
{
	uint32_t count = 0;
	uint8_t writeData;
	uint8_t readData = 0xff;

	if((NULL == mpu6050)
		|| (NULL == mpu6050->I2CReadReg)
		|| (NULL == mpu6050->I2CWriteReg)
	)
	{
		return E_MPU6050_ERROR_NULL;
	}

	//寻找设备设置
	do
	{
		writeData = 0xa5;
		mpu6050->I2CWriteReg(mpu6050->DevAddr, POWER_MANAGEMENT1, &writeData, 1);      //MPU6050电源管理
		mpu6050->I2CReadReg(mpu6050->DevAddr, POWER_MANAGEMENT1, &readData, 1);

		count++;
		if(count > 5)
		{
			return E_MPU6050_ERROR_NOT_FIND_DEV;
		}
	}
	while (readData != 0xa5);
	

	writeData = 0x00;
	mpu6050->I2CWriteReg(mpu6050->DevAddr, POWER_MANAGEMENT1, &writeData, 1);      //MPU6050电源管理

	writeData = 0x00;
	mpu6050->I2CWriteReg(MPU6050_ID, SAMPLE_RATE_DIVIDER, &writeData, 1);   //陀螺仪采样频率

	writeData = 0x03;
	mpu6050->I2CWriteReg(MPU6050_ID, CONFIGURATION, &writeData, 1);         //低通滤波 
		
	//陀螺仪量程  0<<3 250度/s  1<<3 500度/s 2<<3 1000度/s 3<<3 2000度/s
	writeData = mpu6050->GyroRange << 3;
	mpu6050->I2CWriteReg(MPU6050_ID,GYROSCOPE_CONFIGURATION, &writeData, 1);
	
	//加速度量程 0<<3 2g   1<<3 4g   2<<3 8g   3<<3 16g
	writeData = mpu6050->AccRange << 3;
	mpu6050->I2CWriteReg(MPU6050_ID,ACCELEROMETER_CONFIGURATION, &writeData, 1);
}


//获得所有轴的角速度
/*
void mpu6050_get_gyro(Mpu6050_Data_t * Gyro)
{
	uint8_t dat[6] = {0};
	simulation_i2c_readregs(MPU6050_ID,GYRO_XOUT_H,6,dat);

	Gyro->x = (int16_t)((dat[0] << 8) | dat[1]);
	Gyro->y = (int16_t)((dat[2] << 8) | dat[3]);
	Gyro->z = (int16_t)((dat[4] << 8) | dat[5]);

	//Gyro->x = -Gyro->x;
	Gyro->y = -Gyro->y;
	Gyro->z = -Gyro->z;
}
*/

E_MPU6050_ERROR MPU6050_GetBaseGyro(MPU6050_t * mpu6050, MPU6050_BaseData_t * gyro)
{
	uint8_t data[6] = {0};

	if((NULL == mpu6050)
		|| (NULL == mpu6050->I2CReadReg)
	)
	{
		return E_MPU6050_ERROR_NULL;
	}

	
	mpu6050->I2CReadReg(mpu6050->DevAddr, GYRO_XOUT_H, 6, data);

	gyro->x = (int16_t)((uint16_t)(data[0] << 8) | data[1]) - mpu6050->GyroZero.X;
	gyro->y = (int16_t)((uint16_t)(data[2] << 8) | data[3]) - mpu6050->GyroZero.Y;
	gyro->z = (int16_t)((uint16_t)(data[4] << 8) | data[5]) - mpu6050->GyroZero.Z;

	return E_MPU6050_ERROR_OK;
}


//获得所有轴的加速度
E_MPU6050_ERROR MPU6050_GetBaseAcc(MPU6050_t * mpu6050, MPU6050_BaseData_t * acc)
{
	uint8_t data[6] = {0};

	if((NULL == mpu6050)
		|| (NULL == mpu6050->I2CReadReg)
	)
	{
		return E_MPU6050_ERROR_NULL;
	}

	
	mpu6050->I2CReadReg(mpu6050->DevAddr, ACCEL_XOUT_H, 6, data);

	acc->X = (int16_t)((uint16_t)(data[0] << 8) | data[1]);
	acc->Y = (int16_t)((uint16_t)(data[2] << 8) | data[3]);
	acc->Z = (int16_t)((uint16_t)(data[4] << 8) | data[5]);

	return E_MPU6050_ERROR_OK;
}


E_MPU6050_ERROR MPU6050_ConvertDataGyro(MPU6050_t * mpu6050, MPU6050_BaseData_t * in, MPU6050_ConvertData_t * out)
{
	float gyroConvertBase;

	if(NULL == mpu6050)
	{
		return E_MPU6050_ERROR_NULL;
	}

	gyroConvertBase = (float)(32768f / (250u << (mpu6050->GyroRange + 1)));

	out->X = (float)in->X/gyroConvertBase;
	out->Y = (float)in->Y/gyroConvertBase;
	out->Z = (float)in->Z/gyroConvertBase;

	return E_MPU6050_ERROR_OK;
}


E_MPU6050_ERROR MPU6050_ConvertDataAcc(MPU6050_t * mpu6050, MPU6050_BaseData_t * in, MPU6050_ConvertData_t * out)
{
	float accConvertBase;


	if(NULL == mpu6050)
	{
		return E_MPU6050_ERROR_NULL;
	}

	accConvertBase = (float)(32768u >> (mpu6050->AccRange + 1);

	out->X = (float)in->X/accConvertBase;
	out->Y = (float)in->Y/accConvertBase;
	out->Z = (float)in->Z/accConvertBase;

	return E_MPU6050_ERROR_OK;
}


/*
void mpu6050_biquad_fiter_parameter_init()
{
	uint32_t i;
	for(i = 0;i < 3;i++)
	{
		biquad_filter_init_lpf(&gyro_biquad_parameter[i], 500, 70);
		biquad_filter_init_lpf(&acc_biquad_parameter[i], 500, 15);		
	}


}*/



/*
void gyro_data_change()
{
	

	Acc.x = (float)Mpu6050_Acc.x/4096;
	Acc.y = (float)Mpu6050_Acc.y/4096;
	Acc.z = (float)Mpu6050_Acc.z/4096;
	

	Acc.x = biquad_filter(&acc_biquad_parameter[0], Acc.x);
	Acc.y = biquad_filter(&acc_biquad_parameter[1], Acc.y);
	Acc.z = biquad_filter(&acc_biquad_parameter[2], Acc.z);


	
	Gyro.x=(float)(Mpu6050_Gyro.x - Mpu6050_Gyro_Zero.x)/16.384;     //陀螺仪采集零偏和转换单位
	Gyro.y=(float)(Mpu6050_Gyro.y - Mpu6050_Gyro_Zero.y)/16.384;
	Gyro.z=(float)(Mpu6050_Gyro.z - Mpu6050_Gyro_Zero.z)/16.384;


	Gyro.x = biquad_filter(&gyro_biquad_parameter[0], Gyro.x);
	Gyro.y = biquad_filter(&gyro_biquad_parameter[1], Gyro.y);
	Gyro.z = biquad_filter(&gyro_biquad_parameter[2], Gyro.z);

	
}
*/




