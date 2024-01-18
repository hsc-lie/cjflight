#include "sensors.h"

#include "filter.h"

#include "mpu6050_cfg.h"

#include "timer_dev.h"


QueueHandle_t SensorsDataMutex;
static SensorsData_t SensorsData = {0};

static biquadFilter_t biquad_GyroParameterX;
static biquadFilter_t biquad_GyroParameterY;
static biquadFilter_t biquad_GyroParameterZ;
static biquadFilter_t biquad_AccParameterX;
static biquadFilter_t biquad_AccParameterY;
static biquadFilter_t biquad_AccParameterZ;

//姿态四元数
static Quaternion_t Quaternion;

//用于姿态解算中通过加速度计和磁力计修正角度
static QuaternionPIParams_t QuaternionPIParams;


#if 0



PID_t PID_VelocityZ = {0};
PID_t PID_PositionZ = {0};



biquadFilter_t position_z_d_ltem_biquad_parameter = {0};


#define GRAVITY_ACC         980


TriaxialData_t estimate_velocity = {0};
TriaxialData_t estimate_position = {0};
TriaxialData_t estimate_acc = {0};

TriaxialData_t Origion_NamelessQuad_velocity = {0};
TriaxialData_t Origion_NamelessQuad_position = {0};
TriaxialData_t Origion_NamelessQuad_acc = {0};

#define ALTITUDE_SLIDING_FILTER_SIZE    5
float AltitudeSlidingFilterBuffer[ALTITUDE_SLIDING_FILTER_SIZE] = {0};

SlidingFilter_t AltitudeSlidingFilterParam = 
{
	.Index = 0,
	.BufferSize = ALTITUDE_SLIDING_FILTER_SIZE,
	.Buffer = AltitudeSlidingFilterBuffer,
	.Sum = 0,
};

LowPassFilter_t AltitudeLPFParam = 
{
	.K = 0.5,
	.LastValue = 0,
};

float PressureToAltitude(float pressure)//cm
{ 
	return (1.0f - powf(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
}

#define TIME_CONTANST_Z     5.0f
#define K_ACC_Z (5.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_VEL_Z (3.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_POS_Z (3.0f / TIME_CONTANST_Z)

float  High_Filter[3]={
	0.010,  //0.03  0.015
	0.05,//0.05     
	0.02,//0.03 0.02
};


float Altitude_Dealt=0;

float acc_correction[3] = {0};
float vel_correction[3] = {0};
float pos_correction[3] = {0};

float SpeedDealt[3] = {0};

 

void Strapdown_INS_High(TriaxialData_t * acc, float altitude, float dt)
{
	TriaxialData_t accEarth = {0};
	
	QuaternionBodyToEarth(&Quaternion, acc, &accEarth);

	accEarth.X *= GRAVITY_ACC;
	accEarth.Y *= GRAVITY_ACC;
	accEarth.Z *= GRAVITY_ACC;
	
	accEarth.Z -= GRAVITY_ACC + 34;

	Altitude_Dealt = altitude - estimate_position.Z;//气压计(超声波)与SINS估计量的差，单位cm 
	
	if(ABS(Altitude_Dealt) > 500)
	{
		//estimate_position.Z = altitude;
		Origion_NamelessQuad_position.Z = altitude;
	}

	acc_correction[2] += High_Filter[0]*Altitude_Dealt* K_ACC_Z;//加速度校正量
	vel_correction[2] += High_Filter[1]*Altitude_Dealt* K_VEL_Z;//速度校正量
	pos_correction[2] += High_Filter[2]*Altitude_Dealt* K_POS_Z;//位置校正量

	//原始加速度+加速度校正量=融合后的加速度
	estimate_acc.Z = accEarth.Z + acc_correction[2];

	//融合后的加速度积分得到速度增量
	SpeedDealt[2]=estimate_acc.Z * dt;

	//得到速度增量后，更新原始位置
	Origion_NamelessQuad_position.Z += (estimate_velocity.Z + 0.5 * SpeedDealt[2]) * dt,

	//原始位置+位置校正量=融合后位置
	estimate_position.Z = Origion_NamelessQuad_position.Z + pos_correction[2];

	//原始速度+速度校正量=融合后的速度
	Origion_NamelessQuad_velocity.Z += SpeedDealt[2];
	estimate_velocity.Z = Origion_NamelessQuad_velocity.Z + vel_correction[2];
}


float baro_compensate_k = 0.02f;

void PositionFusion(TriaxialData_t * acc, float observeAltitude, float dt)
{
	TriaxialData_t accEarth = {0};
	float altitudeDeviation;
	
	QuaternionBodyToEarth(&Quaternion, acc, &accEarth);

	accEarth.X *= GRAVITY_ACC;
	accEarth.Y *= GRAVITY_ACC;
	accEarth.Z *= GRAVITY_ACC;
	
	accEarth.Z -= GRAVITY_ACC + 15;

	estimate_acc.X = accEarth.X;
	estimate_acc.Y = accEarth.Y;
	estimate_acc.Z = accEarth.Z;
	

	altitudeDeviation = observeAltitude - estimate_position.Z;
	
	//偏差过大 直接给值
	if(ABS(altitudeDeviation) > 500)
	{
		estimate_position.Z = observeAltitude;
		estimate_velocity.Z = 0;
	}
	else
	{
		
		estimate_position.Z += estimate_velocity.Z * dt + 0.5*estimate_acc.Z*dt*dt;
		estimate_velocity.Z += estimate_acc.Z * dt;

		estimate_position.Z += baro_compensate_k * altitudeDeviation;
		estimate_velocity.Z += baro_compensate_k * altitudeDeviation;
	}

}
#endif



//滤波参数初始化 暂时放这里
static void SensorsFilterInit()
{
	biquad_filter_init_lpf(&biquad_GyroParameterX, 1000, 90);
	biquad_filter_init_lpf(&biquad_GyroParameterY, 1000, 90);
	biquad_filter_init_lpf(&biquad_GyroParameterZ, 1000, 90);

	biquad_filter_init_lpf(&biquad_AccParameterX, 1000, 90);	
	biquad_filter_init_lpf(&biquad_AccParameterY, 1000, 90);	
	biquad_filter_init_lpf(&biquad_AccParameterZ, 1000, 90);

	//biquad_filter_init_lpf(&position_z_d_ltem_biquad_parameter, 1000, 50);
}

static void SensorsUpdate()
{
    MPU6050BaseData_t gyroBaseData = {0};
	MPU6050BaseData_t accBaseData = {0};
	
	MPU6050ConvertData_t gyroConvertData = {0};
	MPU6050ConvertData_t accConvertData = {0};

	TriaxialData_t magData = {0};
	
	AttitudeData_t angle;

	uint32_t startTimerCount;
	uint32_t endTimerCount;
	uint32_t timerDiff;

	TimerDevGetCount(TIMER_TEST, &startTimerCount);

    //MPU6050GetBaseAcc(&MPU6050, &accBaseData);
	//MPU6050GetBaseGyro(&MPU6050, &gyroBaseData);
	MPU6050GetBaseAll(&MPU6050, &accBaseData, &gyroBaseData);
		
    gyroBaseData.X -= -7;
    gyroBaseData.Y -= 54;
    gyroBaseData.Z -= -6;

    MPU6050ConvertDataAcc(&MPU6050, &accBaseData, &accConvertData);
    MPU6050ConvertDataGyro(&MPU6050, &gyroBaseData, &gyroConvertData);

    biquad_filter(&biquad_GyroParameterX, gyroConvertData.X);
    biquad_filter(&biquad_GyroParameterY, gyroConvertData.Y);
    biquad_filter(&biquad_GyroParameterZ, gyroConvertData.Z);

    biquad_filter(&biquad_AccParameterX, accConvertData.X);
    biquad_filter(&biquad_AccParameterY, accConvertData.Y);
    biquad_filter(&biquad_AccParameterZ, accConvertData.Z);
		
    accConvertData.Y = -accConvertData.Y;
    accConvertData.Z = -accConvertData.Z;
    gyroConvertData.Y = -gyroConvertData.Y;
    gyroConvertData.Z = -gyroConvertData.Z;

    //四元数姿态解算
    QuaternionAttitudeAlgorithm(&Quaternion, 
                                &QuaternionPIParams, 
                                (TriaxialData_t *)&accConvertData, 
                                (TriaxialData_t *)&gyroConvertData, 
                                &magData, 
                                &angle, 
                                0.001);

	
	TimerDevGetCount(TIMER_TEST, &endTimerCount);
	timerDiff = endTimerCount - startTimerCount;
	(void)timerDiff;

	xSemaphoreTake(SensorsDataMutex, portMAX_DELAY);

	SensorsData.Gyro.X = gyroConvertData.X;
    SensorsData.Gyro.Y = gyroConvertData.Y;
    SensorsData.Gyro.Z = gyroConvertData.Z;

    SensorsData.Acc.X = accConvertData.X;
    SensorsData.Acc.Y = accConvertData.Y;
    SensorsData.Acc.Z = accConvertData.Z;

    SensorsData.Mag.X = magData.X;
    SensorsData.Mag.Y = magData.Y;
    SensorsData.Mag.Z = magData.Z;

    SensorsData.Angle.Pitch = angle.Pitch;
    SensorsData.Angle.Roll = angle.Roll;
    SensorsData.Angle.Yaw = angle.Yaw;

	xSemaphoreGive(SensorsDataMutex);
} 


void SensorsGetData(SensorsData_t *data)
{
	xSemaphoreTake(SensorsDataMutex, portMAX_DELAY);

    data->Gyro.X = SensorsData.Gyro.X;
    data->Gyro.Y = SensorsData.Gyro.Y;
    data->Gyro.Z = SensorsData.Gyro.Z;

    data->Acc.X = SensorsData.Acc.X;
    data->Acc.Y = SensorsData.Acc.Y;
    data->Acc.Z = SensorsData.Acc.Z;

    data->Mag.X = SensorsData.Mag.X;
    data->Mag.Y = SensorsData.Mag.Y;
    data->Mag.Z = SensorsData.Mag.Z;

    data->Angle.Pitch = SensorsData.Angle.Pitch;
    data->Angle.Roll = SensorsData.Angle.Roll;
    data->Angle.Yaw = SensorsData.Angle.Yaw;

	xSemaphoreGive(SensorsDataMutex);
}

void SensorsTask(void *params)
{
    //TickType_t taskTickCount = xTaskGetTickCount();

    QuaternionInit(&Quaternion);
    QuaternionPIParamsInit(&QuaternionPIParams, 0.4f, 0.000f);
    SensorsFilterInit();

    for(;;)
    {
		SensorsUpdate();
		vTaskDelay(1);
        //vTaskDelayUntil(&taskTickCount,1);

#if 0
		if((timeCount % 10) == 0)
		{					
			//接收bmp180海拔高度 
			/*
			BMP280_GetData(&BMP280, &bmp280Data);
			bmp280Data.Pressure = SlidingFilter(&AltitudeSlidingFilterParam, bmp280Data.Pressure);
			altitude = PressureToAltitude(bmp280Data.Pressure);
			altitude = LowPassFilter(&AltitudeLPFParam, altitude);*/


			//SPL06
			SPL06GetDataAll(&SPL06, &spl06Data);
			//spl06Data.Pressure = SlidingFilter(&AltitudeSlidingFilterParam, spl06Data.Pressure);
			altitude = PressureToAltitude(spl06Data.Pressure);
			//altitude = LowPassFilter(&AltitudeLPFParam, altitude);

			//需要打印的位置信息
			xQueueSend(BaroAltitudeQueue,&altitude,0);
			xQueueSend(AltitudeQueue,&estimate_position.Z,0);
			xQueueSend(AccQueue,&estimate_acc.Z,0);

			//xQueueSend(printf_position_queue,&estimate_position.z,0);
			
			//altitude = 100;
			
			//bmp280_data.altitude = sliding_filter(bmp280_data.altitude,altitude_sliding_filter_buff,ALTITUDE_SLIDING_FILTER_SIZE),
			
			//spl06_get_all_data(),
		}
		
        //预估机体速度和位置
		//get_estimate_position(bmp280_data.altitude);
		//bmp280_data.altitude = 0,
		//Strapdown_INS_High();
		
		//需要打印的位置信息
		//xQueueSend(printf_position_queue,&bmp280_data.altitude,0);
		//xQueueSend(printf_position_queue,&bmp280_data.pressure,0);
		//xQueueSend(printf_position_queue,&estimate_position.z,0);

		//需要打印的速度信息
		//xQueueSend(printf_velocity_queue,&estimate_velocity.z,0);
		
		//需要打印的加速度信息
		//xQueueSend(printf_acc_queue,&estimate_acc.z,0);
		//xQueueSend(printf_acc_queue,&earth_acc.z,0);
		//xQueueSend(printf_acc_queue,&Origion_NamelessQuad_acc.z,0);

		//Strapdown_INS_High(&accConvertData, altitude, 0.002);
		PositionFusion((TriaxialData_t *)&accConvertData, altitude, 0.002);
		
		//四元数姿态解算
		QuaternionAttitudeAlgorithm(&Quaternion, 
		                            &QuaternionPIParams, 
								    (TriaxialData_t *)&accConvertData, 
								    (TriaxialData_t *)&gyroConvertData, 
								    &magData, 
								    &nowAngle, 
								    0.001);
#endif
		
    }
}
