#ifndef _FILTER_H_
#define _FILTER_H_


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


void biquad_filter_init_lpf(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq);
float biquad_filter(biquadFilter_t *filter, float input);

float sliding_filter(float input,float * buff,uint32_t buff_size);

#endif /*_FILTER_H_*/
