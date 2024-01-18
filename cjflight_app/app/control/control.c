#include "control.h"

#include <math.h>

#include "main.h"
#include "sensors.h"
#include "remote.h"
#include "remote_data.h"
#include "quaternion.h"
#include "pid.h"
#include "common.h"
#include "filter.h"

#include "motor_cfg.h"
#include "mpu6050_cfg.h"
#include "bmp280_cfg.h"
#include "spl06_cfg.h"

#include "timer_dev.h"


static biquadFilter_t biquad_PitchDItemParam;
static biquadFilter_t biquad_RollDItemParam;
static biquadFilter_t biquad_YawDItemParam;


static PID_Base_t PID_DItemFilterPitchRate(PID_Base_t in)
{
	return  biquad_filter(&biquad_PitchDItemParam, in);
}

static PID_Base_t PID_DItemFilterRollRate(PID_Base_t in)
{
	return  biquad_filter(&biquad_RollDItemParam, in);
}

static PID_Base_t PID_DItemFilterYawRate(PID_Base_t in)
{
	return  biquad_filter(&biquad_YawDItemParam, in);
}

#define PID_PITCH_ANGLE_P    9.0f  
#define PID_PITCH_ANGLE_I    0.0f
#define PID_PITCH_ANGLE_D    0.0f

#define PID_PITCH_RATE_P     4.8f//3.6
#define PID_PITCH_RATE_I     0.001f
#define PID_PITCH_RATE_D     90.0f

#define PID_ROLL_ANGLE_P     PID_PITCH_ANGLE_P
#define PID_ROLL_ANGLE_I     PID_PITCH_ANGLE_I
#define PID_ROLL_ANGLE_D     PID_PITCH_ANGLE_D

#define PID_ROLL_RATE_P      (0.75 * PID_PITCH_RATE_P)
#define PID_ROLL_RATE_I      (0.75 * PID_PITCH_RATE_I)
#define PID_ROLL_RATE_D      (0.75 * PID_PITCH_RATE_D)

#define PID_YAW_ANGLE_P      PID_PITCH_ANGLE_P
#define PID_YAW_ANGLE_I      PID_PITCH_ANGLE_I
#define PID_YAW_ANGLE_D      PID_PITCH_ANGLE_D

#define PID_YAW_RATE_P       4.8f
#define PID_YAW_RATE_I       0.001f
#define PID_YAW_RATE_D       90.0f

//俯仰角角速度环PID
PID_t PID_PitchRate = 
{
	.P = PID_PITCH_RATE_P,    
	.I = PID_PITCH_RATE_I,
	.D = PID_PITCH_RATE_D, 
	.LastDeviation = 0,
	.ISum = 0,
	.ISumMin = -250,
	.ISumMax = 250,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterPitchRate,
};

//俯仰角角度环PID
PID_t PID_PitchAngle = 
{
	.P = PID_PITCH_ANGLE_P,       
	.I = PID_PITCH_ANGLE_I,
	.D = PID_PITCH_ANGLE_D,     
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -0,
	.ISumMax = 0,
	.OutMin = -720,
	.OutMax = 720,

	.DFilterFunc = NULL,
};

//横滚角角速度环PID
PID_t PID_RollRate = 
{
	.P = PID_ROLL_RATE_P,
	.I = PID_ROLL_RATE_I,
	.D = PID_ROLL_RATE_D,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -250,
	.ISumMax = 250,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterRollRate,
};

//横滚角角度环PID
PID_t PID_RollAngle = 
{
	.P = PID_ROLL_ANGLE_P,
	.I = PID_ROLL_ANGLE_I,
	.D = PID_ROLL_ANGLE_D,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = 0,
	.ISumMax = 0,
	.OutMin = -720,
	.OutMax = 720,

	.DFilterFunc = NULL,
};

//偏航角角速度环PID
PID_t PID_YawRate = 
{
	.P = PID_YAW_RATE_P,
	.I = PID_YAW_RATE_I,
	.D = PID_YAW_RATE_D,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -250,
	.ISumMax = 250,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterYawRate,
};

//偏航角角度环PID
PID_t PID_YawAngle = 
{
	.P = PID_YAW_ANGLE_P,
	.I = PID_YAW_ANGLE_I,
	.D = PID_YAW_ANGLE_D,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = 0,
	.ISumMax = 0,
	.OutMin = -720,
	.OutMax = 720,

	.DFilterFunc = NULL,
};



//滤波参数初始化 暂时放这里
static void ControlFilterInit()
{
	biquad_filter_init_lpf(&biquad_PitchDItemParam, 1000, 90);
	biquad_filter_init_lpf(&biquad_RollDItemParam, 1000, 90);
	biquad_filter_init_lpf(&biquad_YawDItemParam, 1000, 90);
}


//所有电机停转
static void MotorStopAll()
{
	uint32_t i;

	for(i = 0;i < 4;++i)
	{
		MotorOut(&Motor[i], MOTOR_STOP_PWM);
	}
}


//姿态控制
void AttitudeControl(uint32_t throttleOut, AttitudeData_t *setAngle, AttitudeData_t *nowAngle, TriaxialData_t *gyro)
{
	uint32_t i;
	static uint8_t timeCount = 0;


	float yaw_error;

	float  pitchRateOut = 0;
	int32_t pitchOut = 0;

	float rollRateOut = 0;
	int32_t rollOut = 0;

	float yawRateOut = 0;
	int32_t yawOut = 0;
	

	int32_t motorOut[4];

	yaw_error = setAngle->Yaw - nowAngle->Yaw;
	if(yaw_error > 180)
	{
		yaw_error -= 360;
	}
	else if(yaw_error < -180)
	{
		yaw_error += 360;
	}
	
	setAngle->Pitch += PITCH_ZERO;
	setAngle->Roll += ROLL_ZERO;


	++timeCount;
	if(5u == timeCount)
	{
		timeCount = 0;
		pitchRateOut = PID_Control(&PID_PitchAngle, nowAngle->Pitch - setAngle->Pitch);
		rollRateOut = PID_Control(&PID_RollAngle, nowAngle->Roll - setAngle->Roll);
		yawRateOut = PID_Control(&PID_YawAngle, yaw_error); 
	}
	
	pitchOut = (int32_t)PID_Control(&PID_PitchRate, -gyro->Y- pitchRateOut);
	rollOut = (int32_t)PID_Control(&PID_RollRate, -gyro->X- rollRateOut);
	yawOut = (int32_t)PID_Control(&PID_YawRate, gyro->Z- yawRateOut);

	
	motorOut[0] = MOTOR_PWM_MIN + pitchOut - rollOut + yawOut + throttleOut;  //400
	motorOut[1] = MOTOR_PWM_MIN + pitchOut + rollOut - yawOut + throttleOut;     
	motorOut[2] = MOTOR_PWM_MIN - pitchOut - rollOut - yawOut + throttleOut;   //400
	motorOut[3] = MOTOR_PWM_MIN - pitchOut + rollOut + yawOut + throttleOut;

	for(i = 0;i < 4;++i)
	{
		motorOut[i] = IntRange(motorOut[i],MOTOR_PWM_MIN,MOTOR_PWM_MAX);
		MotorOut(&Motor[i], motorOut[i]);
	}
}



/*
//位置控制
uint32_t position_control(float now_altitude, float set_altitude)
{
	uint8_t count = 0;
	int32_t throttle_out = 0;
	static float set_velocity_z = 0;
	
	count++,
	if(count >= 5)
	{
		count = 0,
		set_velocity_z = PID_control(&position_z_pid, now_altitude - set_altitude);
		
	}
	throttle_out = (int32_t)PID_control_biquad(&velocity_z_pid, set_velocity_z - estimate_velocity.z, &position_z_d_ltem_biquad_parameter);
	throttle_out += THROTTLE_BASE;

	if(throttle_out < 0)
	{
		throttle_out = 0;
	}

	return throttle_out;
}

*/


void ControlTask(void * parameters)
{

	//TickType_t taskTickCount = xTaskGetTickCount();
	uint32_t timeCount = 0;
	

	//uint16_t remote_data[IBUS_Channel_MAX] = {0};
	RemoteData_t remoteData = {0};
	int32_t remote_lose_count = 0;
	uint8_t remote_lose_flag = 0;

	float setYaw = 0;
	
	//flight_mode_t last_remote_mode = Stabilize_Mode;

	AttitudeData_t setAngle = {0};

	SensorsData_t sensorsData;

	uint32_t startTimerCount;
	uint32_t endTimerCount;
	uint32_t timerDiff;

	ControlFilterInit();

	for(;;)
	{
		timeCount++;
		if(timeCount > 1000)
		{
			timeCount = 0;
		}
		//time_count_start_us();

		//接收遥控器数据
		
		if(xQueueReceive(RemoteDataToControlQueue,&remoteData,0) == pdFALSE)
		{
			remote_lose_count++;
			if(remote_lose_count > 100)
			{
				remote_lose_count = 0;
				remote_lose_flag = 1;
			}
		}
		else
		{
			remote_lose_count = 0;
			remote_lose_flag = 0;
		}

		SensorsGetData(&sensorsData);


		if((RemoteDataGetRockerValue(&remoteData, REMOTE_DATA_LEFT_ROCKER_Y)  < THROTTLE_DEAD_ZONE) || (1 == remote_lose_flag))
		//if(FALSE)
		{
			//电机停转
			MotorStopAll();
			//清除PID角速度环I项
			PID_ISumClean(&PID_PitchRate);
			PID_ISumClean(&PID_RollRate);
			PID_ISumClean(&PID_YawRate);

			//将设定偏航角设置为当前偏航角，防止起飞旋转
			setYaw = sensorsData.Angle.Yaw;

		}
		else
		{
			setYaw += RemoteDataGetRockerValue(&remoteData, REMOTE_DATA_LEFT_ROCKER_X);
			
			if(setYaw > 180)
			{
				setYaw -= 360;
			}
			else if(setYaw < -180)
			{
				setYaw += 360;
			}

			/*if(remote_data.mode == Auto_Mode)
			{
				if(last_remote_mode != Auto_Mode)
				{
					set_altitude = estimate_position.z;
				}
				remote_data.throttle_out = position_control(estimate_position.z, set_altitude);
			}
			last_remote_mode = remote_data.mode;*/

			TimerDevGetCount(TIMER_TEST, &startTimerCount);

			setAngle.Pitch = RemoteDataGetRockerValue(&remoteData, REMOTE_DATA_RIGHT_ROCKER_Y);
			setAngle.Roll = RemoteDataGetRockerValue(&remoteData, REMOTE_DATA_RIGHT_ROCKER_X);
			setAngle.Yaw = setYaw;
			AttitudeControl(RemoteDataGetRockerValue(&remoteData, REMOTE_DATA_LEFT_ROCKER_Y), &setAngle, &sensorsData.Angle, &sensorsData.Gyro);

			TimerDevGetCount(TIMER_TEST, &endTimerCount);
			timerDiff = endTimerCount - startTimerCount;
			(void)timerDiff;
		}
		
		//vTaskDelayUntil(&taskTickCount,1);
		vTaskDelay(1);
	}
}

