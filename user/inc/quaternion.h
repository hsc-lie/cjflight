#ifndef _QUATERNION_H_
#define _QUATERNION_H_





typedef struct
{
	float x;
	float y;
	float z;
}Triaxial_Data_t;

typedef struct
{
	float pitch;
	float roll;
	float yaw;
}Attitude_Data_t;



void imuUpdate(Triaxial_Data_t * Acc, Triaxial_Data_t * Gyro, Attitude_Data_t * Angle , float dt);

void imu_body_to_Earth(Triaxial_Data_t * body_v,Triaxial_Data_t * earth_v);

#endif /*_QUATERNION_H_*/
