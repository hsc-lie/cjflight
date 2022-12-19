#ifndef _FILTER_H_
#define _FILTER_H_


/****************************************************************************************
本文件代码参考自betaFlight
github网址 https://github.com/betaflight/betaflight
****************************************************************************************/




#include "common.h"


typedef struct
{
    float b0, b1, b2, a1, a2;
    float d1, d2;
} biquadFilter_t;


typedef enum
{
    FILTER_LPF,
    FILTER_NOTCH
} biquadFilterType_e;


typedef struct
{
	float K;
	float LastValue;
}LowPassFilter_t;


typedef struct
{
	uint32_t Index;   //请初始化为0

	uint32_t BufferSize;
	float * Buffer;

	float Sum;    //请初始化为0
	
}SlidingFilter_t;


void biquad_filter_init_lpf(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
float biquad_filter(biquadFilter_t *filter, float input);

//一阶低通滤波
float LowPassFilter(LowPassFilter_t * filter, float in);
//滑动滤波
float SlidingFilter(SlidingFilter_t * filter, float in);


#endif /*_FILTER_H_*/
