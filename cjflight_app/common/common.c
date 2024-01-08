#include "common.h"

/*
 * @函数名  IntRange
 * @用  途  整型限幅
 * @参  数  value:输入值
 *          min:最小值
 *          max:最大值
 * @返回值  限幅后的值
*/
int IntRange(int value, int min, int max)
{
	if(value < min)
	{
		return min;
	}
	else if(value > max)
	{
		return max;
	}
	else
	{
		return value;
	}
}

/*
 * @函数名  FloatRange
 * @用  途  浮点型限幅
 * @参  数  value:输入值
 *          min:最小值
 *          max:最大值
 * @返回值  限幅后的值
*/
float FloatRange(float value, float min, float max)
{
	if(value < min)
	{
		return min;
	}
	else if(value > max)
	{
		return max;
	}
	else
	{
		return value;
	}
}
