#include "control.h"

#include <math.h>
#include <stdint.h>

#include "motor_cfg.h"
#include "mpu6050_cfg.h"
#include "bmp280_cfg.h"
#include "spl06_cfg.h"

#include "remote_data.h"

#include "quaternion.h"
#include "remote_task.h"
#include "pid.h"
#include "common.h"
#include "filter.h"

#include "main.h"

biquadFilter_t biquad_PitchDItemParam = {0};
biquadFilter_t biquad_RollDItemParam = {0};
biquadFilter_t biquad_YawDItemParam = {0};


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



//俯仰角角速度环PID
PID_t PID_PitchRate = 
{
	.P = 3.6,
	.I = 0.02,
	.D = 52,    //18
	.LastDeviation = 0,
	.ISum = 0,
	.ISumMin = -500,
	.ISumMax = 500,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterPitchRate,
};

//俯仰角角度环PID
PID_t PID_PitchAngle = 
{
	.P = 5,       
	.I = 0.0,
	.D = 0.0,     
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -0,
	.ISumMax = 0,
	.OutMin = -300,
	.OutMax = 300,

	.DFilterFunc = NULL,
};

//横滚角角速度环PID
PID_t PID_RollRate = 
{
	.P = 3.6 * 0.75,
	.I = 0.02 * 0.75,
	.D = 52 * 0.75,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -500,
	.ISumMax = 500,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterRollRate,
};

//横滚角角度环PID
PID_t PID_RollAngle = 
{
	.P = 5,
	.I = 0,
	.D = 0,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = 0,
	.ISumMax = 0,
	.OutMin = -300,
	.OutMax = 300,

	.DFilterFunc = NULL,
};

//偏航角角速度环PID
PID_t PID_YawRate = 
{
	.P = 8,
	.I = 0.010,
	.D = 0,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = -500,
	.ISumMax = 500,
	.OutMin = -ATTITUDE_CONTROL_OUT_MAX,
	.OutMax = ATTITUDE_CONTROL_OUT_MAX,

	.DFilterFunc = PID_DItemFilterYawRate,
};

//偏航角角度环PID
PID_t PID_YawAngle = 
{
	.P = 5,
	.I = 0,
	.D = 0,
	.ISum = 0,
	.LastDeviation = 0,
	.ISumMin = 0,
	.ISumMax = 0,
	.OutMin = -300,
	.OutMax = 300,

	.DFilterFunc = NULL,
};


PID_t PID_VelocityZ = {0};
PID_t PID_PositionZ = {0};

biquadFilter_t biquad_GyroParameterX = {0};
biquadFilter_t biquad_GyroParameterY = {0};
biquadFilter_t biquad_GyroParameterZ = {0};

biquadFilter_t biquad_AccParameterX = {0};
biquadFilter_t biquad_AccParameterY = {0};
biquadFilter_t biquad_AccParameterZ = {0};






//姿态四元数
Quaternion_t Quaternion = 
{
	.q0=1.0f,
	.q1=0.0f,
	.q2=0.0f,
	.q3=0.0f,
};

//用于姿态解算中通过加速度计和磁力计修正角度
//P越大回正速度越快
Quaternion_PIOffset_t Quaternion_PIOffset = 
{
	.P = 0.4f,
	.I = 0.000f,

	.exInt = 0,
	.eyInt = 0,
	.ezInt = 0,
};




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


//滤波参数初始化 暂时放这里
void FilterInit()
{


	biquad_filter_init_lpf(&biquad_GyroParameterX, 500, 70);
	biquad_filter_init_lpf(&biquad_GyroParameterY, 500, 70);
	biquad_filter_init_lpf(&biquad_GyroParameterZ, 500, 70);

	
	biquad_filter_init_lpf(&biquad_AccParameterX, 500, 70);	
	biquad_filter_init_lpf(&biquad_AccParameterY, 500, 70);	
	biquad_filter_init_lpf(&biquad_AccParameterZ, 500, 70);	


	biquad_filter_init_lpf(&biquad_PitchDItemParam, 500, 70);
	biquad_filter_init_lpf(&biquad_RollDItemParam, 500, 70);
	biquad_filter_init_lpf(&biquad_YawDItemParam, 500, 70);

	

	biquad_filter_init_lpf(&position_z_d_ltem_biquad_parameter, 500, 50);

}


//所有电机停转
static void MotorStopAll()
{
	uint32_t i;

	for(i = 0;i < 4;++i)
	{
		Motor_Out(&Motor[i], MOTOR_STOP_PWM);
	}
}


//姿态控制
void AttitudeControl(uint32_t throttleOut, AttitudeData_t * setAngle, AttitudeData_t * nowAngle, MPU6050_ConvertData_t * gyro)
{
	uint32_t i;
	static uint8_t attitude_control_count = 0;


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


	attitude_control_count++;
	if(1 == attitude_control_count)
	{
		pitchRateOut = PID_Control(&PID_PitchAngle, nowAngle->Pitch - setAngle->Pitch);
	}
	else if(2 == attitude_control_count)
	{
		rollRateOut = PID_Control(&PID_RollAngle, nowAngle->Roll - setAngle->Roll);
	}
	else if(3 == attitude_control_count)
	{
		attitude_control_count = 0;
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
		motorOut[i] = int_range(motorOut[i],MOTOR_PWM_MIN,MOTOR_PWM_MAX);
		Motor_Out(&Motor[i], motorOut[i]);
	}

	
}

/* Workaround a lack of optimization in gcc */
float exp_cst1 = 2139095040.f;
float exp_cst2 = 0.f;


/* Relative error bounded by 1e-5 for normalized outputs
   Returns invalid outputs for nan inputs
   Continuous error */
float exp_approx(float val) {
  union { int32_t i; float f; } xu, xu2;
  float val2, val3, val4, b;
  int32_t val4i;
  val2 = 12102203.1615614f*val+1065353216.f;
  val3 = val2 < exp_cst1 ? val2 : exp_cst1;
  val4 = val3 > exp_cst2 ? val3 : exp_cst2;
  val4i = (int32_t) val4;
  xu.i = val4i & 0x7F800000;                   // mask exponent  / round down to neareset 2^n (implicit mantisa bit)
  xu2.i = (val4i & 0x7FFFFF) | 0x3F800000;     // force exponent to 0
  b = xu2.f;

  /* Generated in Sollya with:
     > f=remez(1-x*exp(-(x-1)*log(2)),
               [|(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x|],
               [1.000001,1.999999], exp(-(x-1)*log(2)));
     > plot(exp((x-1)*log(2))/(f+x)-1, [1,2]);
     > f+x;
  */
  return
    xu.f * (0.509871020343597804469416f + b *
            (0.312146713032169896138863f + b *
             (0.166617139319965966118107f + b *
              (-2.19061993049215080032874e-3f + b *
               1.3555747234758484073940937e-2f))));
}


/* Absolute error bounded by 1e-6 for normalized inputs
   Returns a finite number for +inf input
   Returns -inf for nan and <= 0 inputs.
   Continuous error. */
float log_approx(float val) {
  union { float f; int32_t i; } valu;
  float exp, addcst, x;
  valu.f = val;
  exp = valu.i >> 23;
  /* 89.970756366f = 127 * log(2) - constant term of polynomial */
  addcst = val > 0 ? -89.970756366f : -(float)INFINITY;
  valu.i = (valu.i & 0x7FFFFF) | 0x3F800000;
  x = valu.f;


  /* Generated in Sollya using:
    > f = remez(log(x)-(x-1)*log(2),
            [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x,
              (x-1)*(x-2)*x*x*x|], [1,2], 1, 1e-8);
    > plot(f+(x-1)*log(2)-log(x), [1,2]);
    > f+(x-1)*log(2)
 */
  return
    x * (3.529304993f + x * (-2.461222105f +
      x * (1.130626167f + x * (-0.288739945f +
        x * 3.110401639e-2f))))
    + (addcst + 0.69314718055995f*exp);
}


float pow_approx(float a, float b)
{
    return exp_approx(b * log_approx(a));
}


float PressureToAltitude(float pressure)//cm
{ 
    //return 4433000.0f * (1.0f - powf(0.190295f, pressure/101325.0f));
	//return 44300.0f * (1.0f - powf(pressure/101325.0f, 0.190257519));
	//return (1.0f - pow_approx(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
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
	
	Quaternion_BodyToEarth(&Quaternion, acc, &accEarth);

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
	
	Quaternion_BodyToEarth(&Quaternion, acc, &accEarth);

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

	TickType_t time = xTaskGetTickCount();
	uint32_t timeCount = 0;
	

	//uint16_t remote_data[IBUS_Channel_MAX] = {0};
	RemoteData_t remoteData = {0};
	int32_t remote_lose_count = 0;
	uint8_t remote_lose_flag = 0;

	float setYaw = 0;
	
	//flight_mode_t last_remote_mode = Stabilize_Mode;


	MPU6050_BaseData_t gyroBaseData = {0};
	MPU6050_BaseData_t accBaseData = {0};
	
	MPU6050_ConvertData_t gyroConvertData = {0};
	MPU6050_ConvertData_t accConvertData = {0};

	TriaxialData_t magData = {0, 0, 0};
	//TriaxialData_t magData = {0, 1, 0};//yaw -90
	//TriaxialData_t magData = {0, -1, 0};//yaw 90
	//TriaxialData_t magData = {1, 1, 1};//

	AttitudeData_t nowAngle = {0};
	AttitudeData_t setAngle = {0};


	//BMP280_Data_t bmp280Data = {0};
	SPL06_Data_t spl06Data = {0};
	static float altitude = 0;

	for(;;)
	{
		timeCount++;
		if(timeCount > 1000)
		{
			timeCount = 0;
		}
		//time_count_start_us();

		//接收遥控器数据
		
		if(xQueueReceive(RemoteDataQueue,&remoteData,0) == pdFALSE)
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

		//获取mpu6050原始数据
		MPU6050_GetBaseAcc(&MPU6050, &accBaseData);
		MPU6050_GetBaseGyro(&MPU6050, &gyroBaseData);
		
		gyroBaseData.X -= -7;
		gyroBaseData.Y -= 54;
		gyroBaseData.Z -= -6;

		MPU6050_ConvertDataAcc(&MPU6050, &accBaseData, &accConvertData);
		MPU6050_ConvertDataGyro(&MPU6050, &gyroBaseData, &gyroConvertData);

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


		if((timeCount % 10) == 0)
		{					
			//接收bmp180海拔高度 
			/*
			BMP280_GetData(&BMP280, &bmp280Data);
			bmp280Data.Pressure = SlidingFilter(&AltitudeSlidingFilterParam, bmp280Data.Pressure);
			altitude = PressureToAltitude(bmp280Data.Pressure);
			altitude = LowPassFilter(&AltitudeLPFParam, altitude);*/


			//SPL06
			SPL06_GetDataAll(&SPL06, &spl06Data);
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
		
		
		//四元数姿态解算
		Quaternion_IMUCalculation(&Quaternion, 
		                          &Quaternion_PIOffset, 
								  (TriaxialData_t *)&accConvertData, 
								  (TriaxialData_t *)&gyroConvertData, 
								  &magData, 
								  &nowAngle, 
								  0.002);


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
		
		if((RemoteData_GetRockerValue(&remoteData, E_REMOTE_DATA_LEFT_ROCKER_Y)  < THROTTLE_DEAD_ZONE) || (1 == remote_lose_flag))
		//if(FALSE)
		{
			//电机停转
			MotorStopAll();
			//清除PID角速度环I项
			PID_ISumClean(&PID_PitchRate);
			PID_ISumClean(&PID_RollRate);
			PID_ISumClean(&PID_YawRate);


			//将设定偏航角设置为当前偏航角，防止起飞旋转
			setYaw = nowAngle.Yaw;

		}
		else
		{
			setYaw += RemoteData_GetRockerValue(&remoteData, E_REMOTE_DATA_LEFT_ROCKER_X);
			
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

			setAngle.Pitch = RemoteData_GetRockerValue(&remoteData, E_REMOTE_DATA_RIGHT_ROCKER_Y);
			setAngle.Roll = RemoteData_GetRockerValue(&remoteData, E_REMOTE_DATA_RIGHT_ROCKER_X);
			setAngle.Yaw = setYaw;
			AttitudeControl(RemoteData_GetRockerValue(&remoteData, E_REMOTE_DATA_LEFT_ROCKER_Y), &setAngle, &nowAngle, &gyroConvertData);
		}
		
		vTaskDelayUntil(&time,2);
	}
}

