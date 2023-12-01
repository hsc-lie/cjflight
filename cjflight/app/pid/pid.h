#ifndef __PID_H
#define __PID_H

#include "common.h"

typedef  float  PID_Base_t;      
typedef PID_Base_t (*PID_DFilterFunc_t)(PID_Base_t in);


typedef struct
{
	PID_Base_t P;
	PID_Base_t I;
	PID_Base_t D;
		
	PID_Base_t LastDeviation;
		
	PID_Base_t ISum;
	PID_Base_t ISumMin;
	PID_Base_t ISumMax;
		
	PID_Base_t OutMin;
	PID_Base_t OutMax;

	PID_DFilterFunc_t DFilterFunc;
	
}PID_t;


PID_Base_t PD_Control(PID_t * const pid, PID_Base_t deviation);
PID_Base_t PI_Control(PID_t * const pid, PID_Base_t deviation);
PID_Base_t PID_Control(PID_t * const pid, PID_Base_t deviation);

void PID_ISumClean(PID_t * const pid);




#endif /*__PID_H*/
