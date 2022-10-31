#include "quaternion.h"
#include "math.h"


static float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

static float rMat[3][3]; //旋转矩阵
static float Kp = 0.4f;	     //0.4f	
static float Ki = 0.001f;		  //0.001f
static float exInt = 0.0f;
static float eyInt = 0.0f;
static float ezInt = 0.0f;	

#define DEG2RAD		0.017453293f	//度转弧
#define RAD2DEG		57.29578f		//弧转度


//快速开平方求导
float invSqrt(float x)	
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//更新旋转矩阵
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}




void imuUpdate(Triaxial_Data_t * Acc, Triaxial_Data_t * Gyro, Attitude_Data_t * Angle , float dt)
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;

	
	Gyro->x = Gyro->x * DEG2RAD;	
	Gyro->y = Gyro->y * DEG2RAD;
	Gyro->z = Gyro->z * DEG2RAD;

	if((Acc->x != 0.0f) || (Acc->y != 0.0f) || (Acc->z != 0.0f))
	{
		
		normalise = invSqrt(Acc->x * Acc->x + Acc->y * Acc->y + Acc->z * Acc->z);
		Acc->x *= normalise;
		Acc->y *= normalise;
		Acc->z *= normalise;

		
		ex = (Acc->y * rMat[2][2] - Acc->z * rMat[2][1]);
		ey = (Acc->z * rMat[2][0] - Acc->x * rMat[2][2]);
		ez = (Acc->x * rMat[2][1] - Acc->y * rMat[2][0]);
		
	
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		
		Gyro->x += Kp * ex + exInt;
		Gyro->y += Kp * ey + eyInt;
		Gyro->z += Kp * ez + ezInt;
	}
	
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * Gyro->x - q2Last * Gyro->y - q3Last * Gyro->z) * halfT;
	q1 += ( q0Last * Gyro->x + q2Last * Gyro->z - q3Last * Gyro->y) * halfT;
	q2 += ( q0Last * Gyro->y - q1Last * Gyro->z + q3Last * Gyro->x) * halfT;
	q3 += ( q0Last * Gyro->z + q1Last * Gyro->y - q2Last * Gyro->x) * halfT;
	
	
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	
	
	
	Angle->pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	Angle->roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	Angle->yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;

	Gyro->x = Gyro->x * RAD2DEG;	
	Gyro->y = Gyro->y * RAD2DEG;
	Gyro->z = Gyro->z * RAD2DEG;
	
	
}


void imu_body_to_Earth(Triaxial_Data_t * body_v,Triaxial_Data_t * earth_v)
{
	earth_v->x = rMat[0][0] * body_v->x + rMat[0][1] * body_v->y + rMat[0][2] * body_v->z;
	earth_v->y = rMat[1][0] * body_v->x + rMat[1][1] * body_v->y + rMat[1][2] * body_v->z;
	earth_v->z = rMat[2][0] * body_v->x + rMat[2][1] * body_v->y + rMat[2][2] * body_v->z;
}





