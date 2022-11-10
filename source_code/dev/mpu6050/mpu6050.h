#ifndef __MPU6050_H
#define __MPU6050_H


#include "at32f4xx.h"

#include "quaternion.h"
#include "filter.h"





#define MPU6050_ID                0x68       //MPU6050的id

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



typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	
}Mpu6050_Data_t;



typedef struct 
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float delay_element_1;
	float delay_element_2;
} low_pass_filter2_parameter_t;



extern Attitude_Data_t Angle;

extern Mpu6050_Data_t Mpu6050_Acc;
extern Mpu6050_Data_t Mpu6050_Gyro;

extern Triaxial_Data_t Gyro;
extern Triaxial_Data_t Acc;



void i2c_config(void);

void mpu6050_init(void);
void mpu6050_biquad_fiter_parameter_init(void);

uint8_t mpu6050_get_zero(void);
void gyro_data_change(void);
void mpu6050_get_gyro(Mpu6050_Data_t * Gyro);
void mpu6050_get_acc(Mpu6050_Data_t * Acc);

void biquad_fiter_parameter_init(void);


#endif /*__MPU6050_H_*/
