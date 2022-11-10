#include "pid.h"



float PD_control(PID_t *pid, float error)
{
	float out;
	pid->last_error = pid->now_error;
	pid->now_error = error;
	out = pid->p * pid->now_error + pid->d * (pid->now_error - pid->last_error);
	
	out = float_range(out, pid->out_min, pid->out_max);
	
	return out;
}

float PI_control(PID_t *pid, float error)
{
	float out;
	
	pid->now_error = error;
	pid->i_sum += pid->i * pid->now_error;
	pid->i_sum = float_range(pid->i_sum, pid->i_sum_min, pid->i_sum_max);
	
	out = pid->p * pid->now_error + pid->i_sum;
	out = float_range(out, pid->out_min, pid->out_max);

	return out;
}

float PID_control(PID_t *pid, float error)
{
	float out;
	
	pid->last_error = pid->now_error;
	pid->now_error = error;
	
	pid->i_sum += pid->i * pid->now_error;
	pid->i_sum = float_range(pid->i_sum, pid->i_sum_min, pid->i_sum_max);
	
	out = pid->p * pid->now_error + pid->i_sum + pid->d * (pid->now_error - pid->last_error);
	out = float_range(out, pid->out_min, pid->out_max);

	return out;
}


/*d项带双二阶低通滤波*/
float PID_control_biquad(PID_t *pid, float error, biquadFilter_t *filter)
{
	float out;
	float d_ltem;
	
	pid->last_error = pid->now_error;
	pid->now_error = error;

	d_ltem = (pid->now_error - pid->last_error);

	d_ltem = biquad_filter(filter, d_ltem);
	
	pid->i_sum += pid->i * pid->now_error;
	pid->i_sum = float_range(pid->i_sum, pid->i_sum_min, pid->i_sum_max);
	
	out = pid->p * pid->now_error + pid->i_sum + pid->d * d_ltem;
	out = float_range(out, pid->out_min, pid->out_max);

	return out;
}


void PID_i_sum_clean(PID_t * pid)
{
	pid->i_sum = 0;
}

