#include "quaternion.h"
#include "math.h"




#define DEG_TO_RAD(deg)		((deg) * 0.017453293f)	//度转弧
#define RAD_TO_DEG(rad)		((rad) * 57.29578f)		//弧转度



//快速开平方求导
static float InvSqrt(float x)	
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
void Quaternion_ComputeRotationMatrix(Quaternion_t * quaternion)
{
    float q1q1 = quaternion->q1 * quaternion->q1;
    float q2q2 = quaternion->q2 * quaternion->q2;
    float q3q3 = quaternion->q3 * quaternion->q3;

    float q0q1 = quaternion->q0 * quaternion->q1;
    float q0q2 = quaternion->q0 * quaternion->q2;
    float q0q3 = quaternion->q0 * quaternion->q3;
    float q1q2 = quaternion->q1 * quaternion->q2;
    float q1q3 = quaternion->q1 * quaternion->q3;
    float q2q3 = quaternion->q2 * quaternion->q3;

    quaternion->RotationMatrix[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    quaternion->RotationMatrix[0][1] = 2.0f * (q1q2 + -q0q3);
    quaternion->RotationMatrix[0][2] = 2.0f * (q1q3 - -q0q2);

    quaternion->RotationMatrix[1][0] = 2.0f * (q1q2 - -q0q3);
    quaternion->RotationMatrix[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    quaternion->RotationMatrix[1][2] = 2.0f * (q2q3 + -q0q1);

    quaternion->RotationMatrix[2][0] = 2.0f * (q1q3 + -q0q2);
    quaternion->RotationMatrix[2][1] = 2.0f * (q2q3 - -q0q1);
    quaternion->RotationMatrix[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}


void Quaternion_Update(Quaternion_t quaternion, Triaxial_Data_t * Gyro)
{
	

}


void Quaternion_GetPIOffset(Quaternion_t * quaternion, Quaternion_PIOffset_t * pi, TriaxialData_t * acc, TriaxialData_t * offset, float dt)
{
	float ex; 
	float ey; 
	float ez;
	float normalise;

	if(NULL == quaternion)
		|| (NULL == pi)
		|| (NULL == acc)
		|| (NULL == offset)
	{
		return;	
	}
	

	if((acc->X != 0.0f) || (acc->Y != 0.0f) || (acc->Z != 0.0f))
	{

		
		normalise = InvSqrt(acc->X * acc->X + acc->Y * acc->Y + acc->Z * acc->Z);
		acc->X *= normalise;
		acc->Y *= normalise;
		acc->Z *= normalise;

		
		ex = (acc->Y * quaternion->RotationMatrix[2][2] - acc->Z * quaternion->RotationMatrix[2][1]);
		ey = (acc->Z * quaternion->RotationMatrix[2][0] - acc->X * quaternion->RotationMatrix[2][2]);
		ez = (acc->X * quaternion->RotationMatrix[2][1] - acc->Y * quaternion->RotationMatrix[2][0]);
		
	
		pi->exInt += pi->I * ex * dt;  
		pi->eyInt += pi->I * ey * dt;
		pi->ezInt += pi->I * ez * dt;
		
		
		offset->X += pi->P * ex + pi->exInt;
		offset->Y += pi->P * ey + pi->eyInt;
		offset->Z += pi->P * ez + pi->ezInt;
	}

}


void Quaternion_Update(Quaternion_t * quaternion, TriaxialData_t * acc, TriaxialData_t * gyro, AttitudeData_t * angle , float dt)


void Quaternion_Update(Quaternion_t * quaternion, TriaxialData_t * acc, TriaxialData_t * gyro, AttitudeData_t * angle , float dt)
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;



	float q0 = quaternion->q0;
	float q1 = quaternion->q1;
	float q2 = quaternion->q2;
	float q3 = quaternion->q3;

	
	gyro->X = gyro->X * DEG2RAD;	
	gyro->Y = gyro->Y * DEG2RAD;
	gyro->Z = gyro->Z * DEG2RAD;

	if((acc->X != 0.0f) || (acc->Y != 0.0f) || (acc->Z != 0.0f))
	{
		
		normalise = invSqrt(acc->X * acc->X + acc->Y * acc->Y + acc->Z * acc->Z);
		acc->x *= normalise;
		acc->y *= normalise;
		acc->z *= normalise;

		
		ex = (acc->Y * quaternion->RotationMatrix[2][2] - acc->Z * quaternion->RotationMatrix[2][1]);
		ey = (acc->Z * quaternion->RotationMatrix[2][0] - acc->X * quaternion->RotationMatrix[2][2]);
		ez = (acc->X * quaternion->RotationMatrix[2][1] - acc->Y * quaternion->RotationMatrix[2][0]);
		
	
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		
		gyro->x += Kp * ex + exInt;
		gyro->y += Kp * ey + eyInt;
		gyro->z += Kp * ez + ezInt;
	}
	
	
	q0 += (-q1 * gyro->X - q2 * gyro->Y - q3 * gyro->Z) * halfT;
	q1 += ( q0 * gyro->X + q2 * gyro->Z - q3 * gyro->Y) * halfT;
	q2 += ( q0 * gyro->Y - q1 * gyro->Z + q3 * gyro->X) * halfT;
	q3 += ( q0 * gyro->Z + q1 * gyro->Y - q2 * gyro->X) * halfT;
	
	
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;

	quaternion->q0 = q0;
	quaternion->q1 = q1;
	quaternion->q2 = q2;
	quaternion->q3 = q3;
	
	Quaternion_ComputeRotationMatrix(quaternion);	
	
	
	Angle->pitch = -asinf(quaternion->RotationMatrix[2][0]) * RAD2DEG; 
	Angle->roll = atan2f(quaternion->RotationMatrix[2][1], quaternion->RotationMatrix[2][2]) * RAD2DEG;
	Angle->yaw = atan2f(quaternion->RotationMatrix[1][0], quaternion->RotationMatrix[0][0]) * RAD2DEG;

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





