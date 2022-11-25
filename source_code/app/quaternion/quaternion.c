
#include "quaternion.h"
#include "math.h"



//度转弧
#define DEG_TO_RAD(deg)		((deg) * 0.017453293f)	
//弧转度
#define RAD_TO_DEG(rad)		((rad) * 57.29578f)		



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



//PI互补滤波求补正
static void Quaternion_GetPIGyroOffset(Quaternion_t * quaternion, Quaternion_PIOffset_t * pi, TriaxialData_t * acc, TriaxialData_t * offset, float dt)
{
	float ex = 0; 
	float ey = 0; 
	float ez = 0;
	float normalise = 0;

	if((NULL == quaternion)
		|| (NULL == pi)
		|| (NULL == acc)
		|| (NULL == offset)
	)
	{
		return;	
	}
	
	//虽然浮点型数据都不会直接用恒等于的方式判断是否等于0 但防止后面计算根号0 保险加上这个判断
	if((acc->X != 0.0f) || (acc->Y != 0.0f) || (acc->Z != 0.0f))
	{

		//归一化
		normalise = InvSqrt(acc->X * acc->X + acc->Y * acc->Y + acc->Z * acc->Z);
		acc->X *= normalise;
		acc->Y *= normalise;
		acc->Z *= normalise;
		
		//向量叉乘得出偏差
		ex = (acc->Y * quaternion->RotationMatrix[2][2] - acc->Z * quaternion->RotationMatrix[2][1]);
		ey = (acc->Z * quaternion->RotationMatrix[2][0] - acc->X * quaternion->RotationMatrix[2][2]);
		ez = (acc->X * quaternion->RotationMatrix[2][1] - acc->Y * quaternion->RotationMatrix[2][0]);
		
		//PI互补滤波 I项
		pi->exInt += pi->I * ex * dt;  
		pi->eyInt += pi->I * ey * dt;
		pi->ezInt += pi->I * ez * dt;
		
		//PI互补 P项加上I项
		offset->X += pi->P * ex + pi->exInt;
		offset->Y += pi->P * ey + pi->eyInt;
		offset->Z += pi->P * ez + pi->ezInt;
	}

}

//四元数更新
void Quaternion_Update(Quaternion_t * quaternion, TriaxialData_t * gyro, float dt)
{
	float normalise;
	float halfT = 0.5f * dt;

	float q0 = quaternion->q0;
	float q1 = quaternion->q1;
	float q2 = quaternion->q2;
	float q3 = quaternion->q3;

	//根据角速度更新四元数
	q0 += (-q1 * gyro->X - q2 * gyro->Y - q3 * gyro->Z) * halfT;
	q1 += ( q0 * gyro->X + q2 * gyro->Z - q3 * gyro->Y) * halfT;
	q2 += ( q0 * gyro->Y - q1 * gyro->Z + q3 * gyro->X) * halfT;
	q3 += ( q0 * gyro->Z + q1 * gyro->Y - q2 * gyro->X) * halfT;
	
	//归一化
	normalise = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;

	//最终赋值给外部
	quaternion->q0 = q0;
	quaternion->q1 = q1;
	quaternion->q2 = q2;
	quaternion->q3 = q3;

}


//更新旋转矩阵
static void Quaternion_ComputeRotationMatrix(Quaternion_t * quaternion)
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

	//将四元数更新到旋转矩阵
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


//四元数转欧拉角
void Quaternion_ToAttitudeAngle(Quaternion_t * quaternion, AttitudeData_t * angle)
{
	//先更新旋转矩阵 计算欧拉角是由旋转矩阵计算
	Quaternion_ComputeRotationMatrix(quaternion);

	//由旋转矩阵计算欧拉角
	angle->Pitch = RAD_TO_DEG(-asinf(quaternion->RotationMatrix[2][0])); 
	angle->Roll = RAD_TO_DEG(atan2f(quaternion->RotationMatrix[2][1], quaternion->RotationMatrix[2][2]));
	angle->Yaw = RAD_TO_DEG(atan2f(quaternion->RotationMatrix[1][0], quaternion->RotationMatrix[0][0]));
}


//惯性传感器计算
void Quaternion_IMUCalculation(Quaternion_t * quaternion, Quaternion_PIOffset_t * pi, TriaxialData_t * acc, TriaxialData_t * gyro, AttitudeData_t * angle , float dt)
{
	TriaxialData_t gyroOffset = {0};
	TriaxialData_t gyroRad = {0};

	//角度转弧度
	gyroRad.X = DEG_TO_RAD(gyro->X);	
	gyroRad.Y = DEG_TO_RAD(gyro->Y);
	gyroRad.Z = DEG_TO_RAD(gyro->Z);

	//PI互补滤波
	Quaternion_GetPIGyroOffset(quaternion, pi, acc, &gyroOffset, dt);

	//根据PI互补滤波补正
	gyroRad.X += gyroOffset.X;
	gyroRad.Y += gyroOffset.Y;
	gyroRad.Z += gyroOffset.Z;

	//更新四元数
	Quaternion_Update(quaternion, &gyroRad, dt);

	//四元数更新旋转矩阵
	Quaternion_ComputeRotationMatrix(quaternion);

	//四元数转欧拉角
	Quaternion_ToAttitudeAngle(quaternion, angle);
}



/*
void imu_body_to_Earth(TriaxialData_t * body_v,TriaxialData_t * earth_v)
{
	earth_v->x = rMat[0][0] * body_v->x + rMat[0][1] * body_v->y + rMat[0][2] * body_v->z;
	earth_v->y = rMat[1][0] * body_v->x + rMat[1][1] * body_v->y + rMat[1][2] * body_v->z;
	earth_v->z = rMat[2][0] * body_v->x + rMat[2][1] * body_v->y + rMat[2][2] * body_v->z;
}
*/




