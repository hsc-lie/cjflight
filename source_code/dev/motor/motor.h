#ifndef __MOTOR_H_
#define __MOTOR_H_


#include "common.h"


typedef struct
{
	int32_t OutMin;
	int32_t OutMax;
	
	void (* Out)(int32_t outValue);	
}Motor_t;



extern void Motor_Out(Motor_t * motor, int32_t out);


#endif /*__MOTOR_H_*/
