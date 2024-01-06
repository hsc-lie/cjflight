#ifndef __RING_QUEUE_H_
#define __RING_QUEUE_H_

#include "common.h"

//RingQueueBase_t类型决定队列可以设置的最大长度,根据实际需要设置,最好小于等于CPU位数
typedef unsigned char RingQueueBase_t;
//typedef unsigned short RingQueueBase_t;
//typedef unsigned long RingQueueBase_t;


typedef struct
{
	RingQueueBase_t WriteIndex;
	RingQueueBase_t ReadIndex;
	RingQueueBase_t BufferSize;
	uint8_t *Buffer;
}RingQueue_t;

extern void RingQueueInit(RingQueue_t *queue, uint8_t *buffer, RingQueueBase_t bufferSize);
extern bool RingQueueWriteByte(RingQueue_t *queue, uint8_t data);
extern RingQueueBase_t RingQueueWriteData(RingQueue_t *queue, uint8_t *data, RingQueueBase_t len);
extern bool RingQueueReadByte(RingQueue_t *const queue, uint8_t *data);
extern RingQueueBase_t RingQueueReadData(RingQueue_t *queue, uint8_t *data, RingQueueBase_t len);

#endif /*__RING_QUEUE_H_*/