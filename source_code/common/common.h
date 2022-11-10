#ifndef _COMMON_H_
#define _COMMON_H_

#include "at32f4xx.h"

#define ABS(x)         ((x) < 0 ? (-(x)):(x))


int int_range(int value, int min, int max);
float float_range(float value, float min, float max);



#endif /*_COMMON_H_*/

