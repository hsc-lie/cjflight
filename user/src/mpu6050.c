#include "mpu6050.h"
#include "math.h"
#include "simulation_i2c.h"



#include "filter.h"



#define GYRO_ZERO_COUNT  20        //陀螺仪采集零偏次数

								
		

/**************陀螺仪原始值**************/
Mpu6050_Data_t Mpu6050_Gyro;

/**************陀螺仪单位转换后的值**************/
Triaxial_Data_t Gyro;


/********陀螺仪零偏*********/
Mpu6050_Data_t Mpu6050_Gyro_Zero;

/***************加速度计原始值*************/
Mpu6050_Data_t Mpu6050_Acc;
										   
/***************加速度计单位转换后的值*************/
Triaxial_Data_t Acc;



Attitude_Data_t Angle;


biquadFilter_t gyro_biquad_parameter[3];
biquadFilter_t acc_biquad_parameter[3];





static void mpu6050_delay()
{
	uint32_t i = 100000;
	while(--i);
}


uint8_t mpu6050_get_zero()
{
	uint8_t i;
	int32_t temp1[3] = {0};
	
	int16_t x_min = 30000;
	int16_t x_max = -30000;
	int16_t y_min = 30000;
	int16_t y_max = -30000;
	int16_t z_min = 30000;
	int16_t z_max = -30000;
	
	
	
	   
	/*for (i = 0; i < GYRO_ZERO_COUNT; i++)
	{     
		mpu6050_delay();
		mpu6050_get_gyro(&Mpu6050_Gyro);	// 读取陀螺仪数据
		
		if(x_min > Mpu6050_Gyro.x)
		{
			x_min = Mpu6050_Gyro.x;
		}
		if(x_max < Mpu6050_Gyro.x)
		{
			x_max = Mpu6050_Gyro.x;
		}
		
		if(y_min > Mpu6050_Gyro.y)
		{
			y_min = Mpu6050_Gyro.y;
		}
		if(y_max < Mpu6050_Gyro.y)
		{
			y_max = Mpu6050_Gyro.y;
		}
		
		if(z_min > Mpu6050_Gyro.z)
		{
			z_min = Mpu6050_Gyro.z;
		}
		if(z_max < Mpu6050_Gyro.z)
		{
			z_max = Mpu6050_Gyro.z;
		}
		
		temp1[0] += Mpu6050_Gyro.x;
		temp1[1] += Mpu6050_Gyro.y;
		temp1[2] += Mpu6050_Gyro.z;
	}
	
	if(((x_max - x_min) > 3) || ((y_max - y_min) > 3) || ((z_max - z_min) > 3))
	{
		return 1;
	}
	
	Mpu6050_Gyro_Zero.x = temp1[0] / GYRO_ZERO_COUNT;
	Mpu6050_Gyro_Zero.y = temp1[1] / GYRO_ZERO_COUNT;
	Mpu6050_Gyro_Zero.z = temp1[2] / GYRO_ZERO_COUNT;*/
	
	Mpu6050_Gyro_Zero.x = 0;
	Mpu6050_Gyro_Zero.y = -44;
	Mpu6050_Gyro_Zero.z = 15;
	
	return 0;
	
}


void mpu6050_init()
{
	uint8_t id = 0;
	mpu6050_delay();  //上电延时
	//i2c_config();
	
	
	while(id != 0x98)
	{			
		//检测陀螺仪		
		simulation_i2c_readregs(MPU6050_ID, WHO_AM_I,1,&id);
		mpu6050_delay();
	}
	
	simulation_i2c_writereg(MPU6050_ID,POWER_MANAGEMENT1,0x00);      //MPU6050电源管理
		
	simulation_i2c_writereg(MPU6050_ID,SAMPLE_RATE_DIVIDER,0x00);   //陀螺仪采样频率
		
	simulation_i2c_writereg(MPU6050_ID,CONFIGURATION,0x03);         //低通滤波 
		
	//陀螺仪量程  0<<3 250度/s  1<<3 500度/s 2<<3 1000度/s 3<<3 2000度/s
	simulation_i2c_writereg(MPU6050_ID,GYROSCOPE_CONFIGURATION,3 << 3);
	
	//加速度量程 0<<3 2g   1<<3 4g   2<<3 8g   3<<3 16g
	simulation_i2c_writereg(MPU6050_ID,ACCELEROMETER_CONFIGURATION,2 << 3);
		
	
	
	
	
}




//获得所有轴的角速度
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

//获得所有轴的加速度
void mpu6050_get_acc(Mpu6050_Data_t * Acc)
{
	uint8_t dat[6] = {0};
	simulation_i2c_readregs(MPU6050_ID,ACCEL_XOUT_H,6,dat);

	Acc->x = (int16_t)((dat[0] << 8) | dat[1]);
	Acc->y = (int16_t)((dat[2] << 8) | dat[3]);
	Acc->z = (int16_t)((dat[4] << 8) | dat[5]);

	//Acc->x = -Acc->x;
	Acc->y = -Acc->y;
	Acc->z = -Acc->z;
}



void mpu6050_biquad_fiter_parameter_init()
{
	uint32_t i;
	for(i = 0;i < 3;i++)
	{
		biquad_filter_init_lpf(&gyro_biquad_parameter[i], 500, 70);
		biquad_filter_init_lpf(&acc_biquad_parameter[i], 500, 15);		
	}


}




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





