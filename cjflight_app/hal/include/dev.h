#ifndef __DEV_H_
#define __DEV_H_

#include "common.h"

#define DEV_PARAM_CHECK_CONFIG       TRUE

#if(FALSE != DEV_PARAM_CHECK_CONFIG)

#define DEV_NUM_CHECK(name, num) \
do\
{\
    if((num) >= name##_DEV_NUM_MAX)\
    {\
	    return name##_DEV_ERROR_INVALID_PARAM;\
    }\
}while(0)

#define DEV_BASE_PARAM_CHECK(name, table, num, func) \
do\
{\
    DEV_NUM_CHECK(name, num);\
    if(NULL == table[num])\
    {\
        return name##_DEV_ERROR_UNREGISTERED;\
    }\
    if(NULL == (table[num]->func))\
    {\
        return name##_DEV_ERROR_NULL_FUNC;\
    }\
} while(0)

#else

#define DEV_NUM_CHECK(name, num) 
#define DEV_BASE_PARAM_CHECK(name, num, func) 

#endif

#endif /*__DEV_H_*/