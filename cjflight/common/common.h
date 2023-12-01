#ifndef _COMMON_H_
#define _COMMON_H_

#include "at32f4xx.h"

#define ABS(x)         ((x) < 0 ? (-(x)):(x))


#ifndef NULL
#define NULL  ((void *)0)
#endif



#ifndef FALSE
#define FALSE  (0)
#endif


#ifndef TRUE
#define TRUE   (1)
#endif

#ifndef int8_t
typedef signed char  int8_t; 
#endif

#ifndef uint8_t
typedef unsigned char  uint8_t;
#endif

#ifndef int16_t
typedef signed short  int16_t; 
#endif

#ifndef uint16_t
typedef unsigned short  uint16_t;
#endif

#ifndef int32_t
//typedef signed long  int32_t; 
#endif

#ifndef uint32_t
//typedef unsigned long  uint32_t;
#endif

#ifndef int64_t
typedef signed long long  int64_t; 
#endif

#ifndef uint64_t
typedef unsigned long long  uint64_t;
#endif










int int_range(int value, int min, int max);
float float_range(float value, float min, float max);








#endif /*_COMMON_H_*/

