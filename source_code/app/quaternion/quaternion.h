#ifndef _QUATERNION_H_
#define _QUATERNION_H_



#include "common.h"

typedef struct
{
	float X;
	float Y;
	float Z;
}TriaxialData_t;

typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
}AttitudeData_t;



typedef struct
{
	float P = 0.4f;
	float I = 0.001f;

	float exInt;
	float eyInt;
	float ezInt;


}Quaternion_PIOffset_t;

typedef struct
{
	float q0=1.0f;
	float q1=0.0f;
	float q2=0.0f;
	float q3=0.0f;

	float RotationMatrix[3][3];
}Quaternion_t;



void imuUpdate(Quaternion_t quaternion, TriaxialData_t * Acc, TriaxialData_t * Gyro, AttitudeData_t * Angle , float dt);

void imu_body_to_Earth(Triaxial_Data_t * body_v,Triaxial_Data_t * earth_v);

#endif /*_QUATERNION_H_*/
