#include "motor.h"



void MotorOut(Motor_t * motor, int32_t outValue)
{

	if((NULL != motor)
		&& (NULL != motor->Out)
	)
	{

		if(outValue < motor->OutMin)
		{
			outValue = motor->OutMin;
		}
	
		if(outValue > motor->OutMax)
		{
			outValue = motor->OutMax;
		}

		
		motor->Out(outValue);
	}
}


