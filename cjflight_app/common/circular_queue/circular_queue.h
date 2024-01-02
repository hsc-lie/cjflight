#ifndef __CIRCULAR_QUEUE_H_
#define __CIRCULAR_QUEUE_H_


#include "common.h"

typedef enum
{
	CIRCULAR_QUEUE_STATUS_NOT_FULL = 0,     //队列未满
	CIRCULAR_QUEUE_STATUS_FULL,             //队列已满
}CIRCULAR_QUEUE_STATUS_t;

typedef enum
{
	CIRCULAR_QUEUE_ERROR_OK = 0,    //OK
	CIRCULAR_QUEUE_ERROR_NULL,      //队列指针为NULL
	CIRCULAR_QUEUE_ERROR_FULL,      //队列已满
	CIRCULAR_QUEUE_ERROR_EMPTY,     //队列为空
}CIRCULAR_QUEUE_ERROR_t;

typedef struct
{
	CIRCULAR_QUEUE_STATUS_t Status;

	uint8_t *Buffer;
	uint32_t BufferSize;

	uint32_t WriteIndex;
	uint32_t ReadIndex;
	
}CircularQueue_t;

extern CIRCULAR_QUEUE_ERROR_t CircularQueueWriteByte(CircularQueue_t *const queue, uint8_t data);
extern CIRCULAR_QUEUE_ERROR_t CircularQueueReadByte(CircularQueue_t *const queue, uint8_t *data);

#endif /*__CIRCULAR_QUEUE_H_*/