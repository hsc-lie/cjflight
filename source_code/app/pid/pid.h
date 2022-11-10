#ifndef __PID_H
#define __PID_H

#include "common.h"

#include "mpu6050.h"
#include "filter.h"


typedef struct
{
	float p;
	float i;
	float d;
		
	float now_error;
	float last_error;
		
	float i_sum;
	float i_sum_min;
	float i_sum_max;
		
	float out_min;
	float out_max;
}PID_t;


float PD_control(PID_t * pid, float error);
float PI_control(PID_t *pid, float error);
float PID_control(PID_t *pid, float error);

float PID_control_lpf2(PID_t *pid, float error, low_pass_filter2_parameter_t * lpf2_parameter);
float PID_control_biquad(PID_t *pid, float error, biquadFilter_t *filter);

void PID_i_sum_clean(PID_t * pid);



#endif /*__PID_H*/
