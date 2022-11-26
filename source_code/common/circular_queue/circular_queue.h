#ifndef __CIRCULAR_QUEUE_H_
#define __CIRCULAR_QUEUE_H_


#include "common.h"


typedef enum
{
	E_CIRCULAR_QUEUE_STATUS_NOT_FULL = 0,     //队列未满
	E_CIRCULAR_QUEUE_STATUS_FULL,          //队列已满
}E_CIRCULAR_QUEUE_STATUS;


typedef enum
{
	E_CIRCULAR_QUEUE_ERROR_OK = 0,    //OK
	E_CIRCULAR_QUEUE_ERROR_NULL,      //队列指针为NULL
	E_CIRCULAR_QUEUE_ERROR_FULL,      //队列已满
	E_CIRCULAR_QUEUE_ERROR_EMPTY,     //队列为空
}E_CIRCULAR_QUEUE_ERROR;




typedef struct
{
	E_CIRCULAR_QUEUE_STATUS Status;

	uint8_t * Buffer;
	uint32_t BufferSize;

	uint32_t WriteIndex;
	uint32_t ReadIndex;
	
}CircularQueue_t;


#endif /*__CIRCULAR_QUEUE_H_*/
