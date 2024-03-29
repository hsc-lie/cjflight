#include "quaternion.h"
#include "math.h"

//度转弧
#define DEG_TO_RAD(deg)		((deg) * 0.017453293f)	
//弧转度
#define RAD_TO_DEG(rad)		((rad) * 57.29578f)		

//快速开平方求倒数
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

/*
 * @函数名  QuaternionInit
 * @用  途  四元数初始化
 * @参  数  quaternion:四元数
 * @返回值  
*/
void QuaternionInit(Quaternion_t *quaternion)
{
	quaternion->q0 = 1.0f;
	quaternion->q1 = 0.0f;
	quaternion->q2 = 0.0f;
	quaternion->q3 = 0.0f;
}

/*
 * @函数名  QuaternionPIParamsInit
 * @用  途  四元数PI参数初始化
 * @参  数  params:PI参数
 *          p:P值
 *          i:I值
 * @返回值  
*/
void QuaternionPIParamsInit(QuaternionPIParams_t *params, float p, float i)
{
	params->P = p;
	params->I = i;
	params->exInt = 0.0f;
	params->eyInt = 0.0f;
	params->ezInt = 0.0f;
}

/*
 * @函数名  QuaternionGetPIGyroOffset
 * @用  途  PI互补滤波求补正
 * @参  数  quaternion:四元数
 *          pi:PI参数
 *          acc:输入三轴加速度
 *          mag:输入三轴磁场强度
 *          offset:输出的三轴角速度偏移补正
 *          dt:计算周期(单位s)
 * @返回值  
*/
static void QuaternionGetPIGyroOffset(Quaternion_t *quaternion, QuaternionPIParams_t *pi, TriaxialData_t *acc, TriaxialData_t *mag, TriaxialData_t *offset, float dt)
{
	TriaxialData_t accTemp;
	TriaxialData_t magTemp;

	float ex = 0; 
	float ey = 0; 
	float ez = 0;
	float normalise = 0;

	//磁力在地球坐标系投影
	TriaxialData_t magEarth = {0};
	TriaxialData_t magBody = {0};

	magTemp.X = mag->X;
	magTemp.Y = mag->Y;
	magTemp.Z = mag->Z;

	//虽然浮点型数据都不会直接用恒等于的方式判断是否等于0 但防止后面计算根号0 保险加上这个判断
	if((magTemp.X != 0.0f) || (magTemp.Y != 0.0f) || (magTemp.Z != 0.0f))
	{
		//归一化
		normalise = InvSqrt(magTemp.X * magTemp.X + magTemp.Y * magTemp.Y + magTemp.Z * magTemp.Z);
		magTemp.X *= normalise;
		magTemp.Y *= normalise;
		magTemp.Z *= normalise;
	
		//磁力从机体坐标转到地球坐标
		QuaternionBodyToEarth(quaternion, &magTemp, &magEarth);
		//水平面向量和为北边 勾股定理定北边为x轴 
		magEarth.X = 1.0f / InvSqrt(magEarth.X * magEarth.X + magEarth.Y * magEarth.Y);
		magEarth.Y = 0;


		//磁力从地球坐标转到机体坐标
		QuaternionEarthToBody(quaternion, &magEarth, &magBody);
	}


	accTemp.X = acc->X;
	accTemp.Y = acc->Y;
	accTemp.Z = acc->Z;
	//虽然浮点型数据都不会直接用恒等于的方式判断是否等于0 但防止后面计算根号0 保险加上这个判断
	if((accTemp.X != 0.0f) || (accTemp.Y != 0.0f) || (accTemp.Z != 0.0f))
	{

		//归一化
		normalise = InvSqrt(accTemp.X * accTemp.X + accTemp.Y * accTemp.Y + accTemp.Z * accTemp.Z);
		accTemp.X *= normalise;
		accTemp.Y *= normalise;
		accTemp.Z *= normalise;
		
		//向量叉乘得出偏差
		ex = (accTemp.Y * quaternion->RotationMatrix[2][2] - accTemp.Z * quaternion->RotationMatrix[2][1]) + (magTemp.Y * magBody.Z - magTemp.Z * magBody.Y);
		ey = (accTemp.Z * quaternion->RotationMatrix[2][0] - accTemp.X * quaternion->RotationMatrix[2][2]) + (magTemp.Z * magBody.X - magTemp.X * magBody.Z);
		ez = (accTemp.X * quaternion->RotationMatrix[2][1] - accTemp.Y * quaternion->RotationMatrix[2][0]) + (magTemp.X * magBody.Y - magTemp.Y * magBody.X);
		
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

/*
 * @函数名  QuaternionUpdate
 * @用  途  四元数更新
 * @参  数  quaternion:四元数
 *          gyro:输入三轴角速度
 *          dt:计算周期(单位s)
 * @返回值  
*/
void QuaternionUpdate(Quaternion_t *quaternion, TriaxialData_t *gyro, float dt)
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

/*
 * @函数名  QuaternionRotationMatrixUpdate
 * @用  途  更新旋转矩阵
 * @参  数  quaternion:四元数
 *          angle:输出欧拉角
 * @返回值  
*/
static void QuaternionRotationMatrixUpdate(Quaternion_t *quaternion)
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

/*
 * @函数名  QuaternionToAttitudeAngle
 * @用  途  四元数转欧拉角
 * @参  数  quaternion:四元数
 *          angle:输出欧拉角
 * @返回值  
*/
void QuaternionToAttitudeAngle(Quaternion_t *quaternion, AttitudeData_t *angle)
{
	//先更新旋转矩阵 计算欧拉角是由旋转矩阵计算
	QuaternionRotationMatrixUpdate(quaternion);

	//由旋转矩阵计算欧拉角
	angle->Pitch = RAD_TO_DEG(-asinf(quaternion->RotationMatrix[2][0])); 
	angle->Roll = RAD_TO_DEG(atan2f(quaternion->RotationMatrix[2][1], quaternion->RotationMatrix[2][2]));
	angle->Yaw = RAD_TO_DEG(atan2f(quaternion->RotationMatrix[1][0], quaternion->RotationMatrix[0][0]));
}

/*
 * @函数名  QuaternionAttitudeAlgorithm
 * @用  途  四元数姿态解算
 * @参  数  quaternion:四元数
 *          pi:PI参数
 *          acc:输入三轴加速度
 *          gyro:输入三轴角速度
 *          mag:输入三轴磁场强度
 *          angle:输出欧拉角
 *          dt:计算周期(单位s)
 * @返回值  
*/
void QuaternionAttitudeAlgorithm(Quaternion_t *quaternion, QuaternionPIParams_t *pi, TriaxialData_t *acc, TriaxialData_t *gyro, TriaxialData_t *mag, AttitudeData_t *angle , float dt)
{
	TriaxialData_t gyroOffset = {0};
	TriaxialData_t gyroRad = {0};

	//角度转弧度
	gyroRad.X = DEG_TO_RAD(gyro->X);	
	gyroRad.Y = DEG_TO_RAD(gyro->Y);
	gyroRad.Z = DEG_TO_RAD(gyro->Z);

	//PI互补滤波
	QuaternionGetPIGyroOffset(quaternion, pi, acc, mag, &gyroOffset, dt);

	//根据PI互补滤波补正
	gyroRad.X += gyroOffset.X;
	gyroRad.Y += gyroOffset.Y;
	gyroRad.Z += gyroOffset.Z;

	//更新四元数
	QuaternionUpdate(quaternion, &gyroRad, dt);

	//四元数更新旋转矩阵
	QuaternionRotationMatrixUpdate(quaternion);

	//四元数转欧拉角
	QuaternionToAttitudeAngle(quaternion, angle);
}

/*
 * @函数名  QuaternionBodyToEarth
 * @用  途  机体坐标系转到地球坐标系
 * @参  数  quaternion:四元数
 *          body:输入机体坐标系3轴加速度
 *          earth:输出地球坐标系3轴加速度
 * @返回值  
*/
void QuaternionBodyToEarth(Quaternion_t *quaternion, TriaxialData_t *body, TriaxialData_t *earth)
{
	earth->X = quaternion->RotationMatrix[0][0] * body->X + quaternion->RotationMatrix[0][1] * body->Y + quaternion->RotationMatrix[0][2] * body->Z;
	earth->Y = quaternion->RotationMatrix[1][0] * body->X + quaternion->RotationMatrix[1][1] * body->Y + quaternion->RotationMatrix[1][2] * body->Z;
	earth->Z = quaternion->RotationMatrix[2][0] * body->X + quaternion->RotationMatrix[2][1] * body->Y + quaternion->RotationMatrix[2][2] * body->Z;
}

/*
 * @函数名  QuaternionEarthToBody
 * @用  途  地球坐标系转到机体坐标系
 * @参  数  quaternion:四元数
 *          earth:输入地球坐标系3轴加速度
 *          body:输出机体坐标系3轴加速度
 * @返回值  
*/
void QuaternionEarthToBody(Quaternion_t *quaternion, TriaxialData_t *earth, TriaxialData_t *body)
{
	body->X = quaternion->RotationMatrix[0][0] * earth->X + quaternion->RotationMatrix[1][0] * earth->Y + quaternion->RotationMatrix[2][0] * earth->Z;
	body->Y = quaternion->RotationMatrix[0][1] * earth->X + quaternion->RotationMatrix[1][1] * earth->Y + quaternion->RotationMatrix[2][1] * earth->Z;
	body->Z = quaternion->RotationMatrix[0][2] * earth->X + quaternion->RotationMatrix[1][2] * earth->Y + quaternion->RotationMatrix[2][2] * earth->Z;
}
