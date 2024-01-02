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

//float P = 0.4f;
//float I = 0.001f;
typedef struct
{
	float P;
	float I;

	float exInt;
	float eyInt;
	float ezInt;
}QuaternionPIParams_t;

/*
	定义Quaternion_t结构体时请初始化为以下值
	q0=1.0f;
	q1=0.0f;
	q2=0.0f;
	q3=0.0f;
*/
typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;

	float RotationMatrix[3][3];
}Quaternion_t;

extern void QuaternionUpdate(Quaternion_t *quaternion, TriaxialData_t *gyro, float dt);
extern void QuaternionToAttitudeAngle(Quaternion_t *quaternion, AttitudeData_t *angle);
extern void QuaternionAttitudeAlgorithm(Quaternion_t *quaternion, QuaternionPIParams_t *pi, TriaxialData_t *acc, TriaxialData_t *gyro, TriaxialData_t *mag, AttitudeData_t *angle , float dt);
extern void QuaternionBodyToEarth(Quaternion_t *quaternion, TriaxialData_t *body, TriaxialData_t *earth);
extern void QuaternionEarthToBody(Quaternion_t *quaternion, TriaxialData_t *earth, TriaxialData_t *body);

#endif /*_QUATERNION_H_*/