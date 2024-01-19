#ifndef __COMMON_H_
#define __COMMON_H_

//#include <stdio.h>
//#include <stdint.h>

#define ABS(x)         ((x) < 0 ? (-(x)):(x))

#ifndef NULL
#define NULL  ((void *)0)
#endif


#ifndef FALSE
#define FALSE  (0)
#endif


#ifndef TRUE
#define TRUE   (!(FALSE))
#endif


typedef unsigned char  bool; 
typedef signed char  int8_t; 
typedef unsigned char  uint8_t;
typedef signed short  int16_t; 
typedef unsigned short  uint16_t;
typedef signed long  int32_t; 
typedef unsigned long  uint32_t;
typedef signed long long  int64_t; 
typedef unsigned long long  uint64_t;

extern int IntRange(int value, int min, int max);
extern float FloatRange(float value, float min, float max);

extern uint32_t GetTimeDiff(uint32_t start, uint32_t end, uint32_t max);

#endif /*__COMMON_H_*/