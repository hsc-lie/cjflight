#include "filter.h"
#include "math.h"


/****************************************************************************************
本文件代码参考自betaFlight
github网址 https://github.com/betaflight/betaflight
****************************************************************************************/


#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/
#define M_PIf       3.1415926f

#define sinPolyCoef3 -1.666665710e-1f        // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5  8.333017292e-3f        // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f        // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9  2.600054768e-6f        // Double:  2.600054767890361277123254766503271638682e-6


float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}

/*双二阶滤波配置*/
void biquadFilterInit(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, float Q, biquadFilterType_e filterType)
{
    // Check for Nyquist frequency and if it's not possible to initialize filter as requested - set to no filtering at all
    if (filterFreq < (samplingFreq / 2)) {
        // setup variables
        const float sampleRate = samplingFreq;
        const float omega = 2.0f * M_PIf * ((float)filterFreq) / sampleRate;
        const float sn = sin_approx(omega);
        const float cs = cos_approx(omega);
        const float alpha = sn / (2 * Q);

        float b0, b1, b2;
        switch (filterType) 
        {
            case FILTER_LPF:
                b0 = (1 - cs) / 2;
                b1 = 1 - cs;
                b2 = (1 - cs) / 2;
                break;
            case FILTER_NOTCH:
                b0 =  1;
                b1 = -2 * cs;
                b2 =  1;
                break;
        }
        const float a0 =  1 + alpha;
        const float a1 = -2 * cs;
        const float a2 =  1 - alpha;

        // precompute the coefficients
        filter->b0 = b0 / a0;
        filter->b1 = b1 / a0;
        filter->b2 = b2 / a0;
        filter->a1 = a1 / a0;
        filter->a2 = a2 / a0;
    }
    else 
    {
        // Not possible to filter frequencies above Nyquist frequency - passthrough
        filter->b0 = 1.0f;
        filter->b1 = 0.0f;
        filter->b2 = 0.0f;
        filter->a1 = 0.0f;
        filter->a2 = 0.0f;
    }

    // zero initial samples
    filter->d1 = filter->d2 = 0;
}


/*双二阶低通滤波初始化*/
void biquad_filter_init_lpf(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq)
{
    biquadFilterInit(filter, samplingFreq, filterFreq, BIQUAD_Q, FILTER_LPF);
}

float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff)
{
    const float octaves = log2f((float)centerFreq  / (float)cutoff) * 2;
    return sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1);
}


void biquad_filter_init_notch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, samplingFreq, filterFreq, Q, FILTER_NOTCH);
}


/*双二阶低通滤波*/
float biquad_filter(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->d1;
    filter->d1 = filter->b1 * input - filter->a1 * result + filter->d2;
    filter->d2 = filter->b2 * input - filter->a2 * result;
    return result;
}


/*滑动滤波*/
float sliding_filter(float input,float * buff,uint32_t buff_size)
{
    uint32_t i;
    float out;
    float sum = 0;

    //更新缓存
    for(i = buff_size - 1;i > 0;i--)
    {
        buff[i] = buff[i-1];
        sum += buff[i-1];
    }
    buff[0] = input;
    sum += buff[0];

    out = sum / buff_size;

    return out;
}


float LowPassFilter(LowPassFilter_t * filter, float in)
{
	float out;

	out = filter->K * in + (1 - filter->K) * filter->LastValue;
	filter->LastValue = in;
	
	return out;
}


//滑动滤波
float SlidingFilter(SlidingFilter_t * filter, float in)
{

	uint32_t index;

	if((NULL == filter)
		|| (NULL == filter->Buffer)
	)
	{
		return 0;
	}

	
	index = filter->Index;

	//减去最旧的值
	filter->Sum -= filter->Buffer[index];

	//得到新值
	filter->Sum += in;
	filter->Buffer[index] = in;

	++index;
	if(index == filter->BufferSize)
	{
		index = 0;
	}

	filter->Index = index;

	return (filter->Sum / filter->BufferSize);
}

