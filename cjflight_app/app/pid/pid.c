#include "pid.h"

//限幅
static PID_Base_t PID_Limit(PID_Base_t value, PID_Base_t min, PID_Base_t max)
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



PID_Base_t PD_Control(PID_t * const pid, PID_Base_t deviation)
{
	PID_Base_t out;
	PID_Base_t dItem;

	if(NULL == pid)
	{
		return 0;
	}

	dItem = deviation - pid->LastDeviation;

	if(NULL != pid->DFilterFunc)
	{
		dItem = pid->DFilterFunc(dItem);
	}
	
	out = pid->P * deviation + pid->D * dItem;
	
	out = PID_Limit(out, pid->OutMin, pid->OutMax);

	pid->LastDeviation = deviation;
	
	return out;
}

PID_Base_t PI_Control(PID_t * const pid, PID_Base_t deviation)
{
	PID_Base_t out;

	if(NULL == pid)
	{
		return 0;
	}
	
	pid->ISum += pid->I * deviation;
	pid->ISum = PID_Limit(pid->ISum, pid->ISumMin, pid->ISumMax);
	
	out = pid->P * deviation + pid->ISum;
	out = PID_Limit(out, pid->OutMin, pid->OutMax);

	return out;
}

PID_Base_t PID_Control(PID_t * const pid, PID_Base_t deviation)
{
	PID_Base_t out;
	PID_Base_t dItem;
	
	pid->ISum += pid->I * deviation;
	pid->ISum = PID_Limit(pid->ISum, pid->ISumMin, pid->ISumMax);

	dItem = deviation - pid->LastDeviation;

	if(NULL != pid->DFilterFunc)
	{
		dItem = pid->DFilterFunc(dItem);
	}
	
	out = pid->P * deviation + pid->ISum + pid->D * dItem;
	out = PID_Limit(out, pid->OutMin, pid->OutMax);

	pid->LastDeviation = deviation;

	return out;
}



void PID_ISumClean(PID_t * const pid)
{
	if(NULL != pid)
	{
		pid->ISum = 0;
	}
}

