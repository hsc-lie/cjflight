#include "circular_queue.h"

/*
 * @函数名  CircularQueueWriteByte
 * @用  途  环形队列写入
 * @参  数  queue:环形队列
 *          data:要写入的数据
 * @返回值  错误的状态
*/
CIRCULAR_QUEUE_ERROR_t CircularQueueWriteByte(CircularQueue_t *const queue, uint8_t data)
{
	uint32_t writeIndex;

	if(NULL == queue->Buffer)
	{
		return CIRCULAR_QUEUE_ERROR_NULL;
	}

	writeIndex = queue->WriteIndex;

	if(CIRCULAR_QUEUE_STATUS_FULL == queue->Status)
	{
		return CIRCULAR_QUEUE_ERROR_FULL;
	}
	else
	{
		queue->Buffer[writeIndex] = data;
		++writeIndex;
		if(writeIndex == queue->BufferSize)
		{
			writeIndex = 0;
		}

		queue->WriteIndex = writeIndex;
		
		if((CIRCULAR_QUEUE_STATUS_NOT_FULL == queue->Status)
			&& (writeIndex == queue->ReadIndex)
		)
		{
			queue->Status = CIRCULAR_QUEUE_STATUS_FULL;
		}
		
		return CIRCULAR_QUEUE_ERROR_OK;
	}
}

/*
 * @函数名  CircularQueueReadByte
 * @用  途  环形队列读取
 * @参  数  queue:环形队列
 *          data:读取到的数据
 * @返回值  错误的状态
*/
CIRCULAR_QUEUE_ERROR_t CircularQueueReadByte(CircularQueue_t *const queue, uint8_t *data)
{
	uint32_t readIndex;

	if(NULL == queue->Buffer)
	{
		return CIRCULAR_QUEUE_ERROR_NULL;
	}

	readIndex = queue->ReadIndex;

	if((readIndex == queue->WriteIndex)
		&& (CIRCULAR_QUEUE_STATUS_NOT_FULL == queue->Status)
	)
	{
		return CIRCULAR_QUEUE_ERROR_EMPTY;
	}
	else
	{
		*data = queue->Buffer[readIndex];
	
		++readIndex;
		if(readIndex == queue->BufferSize)
		{
			readIndex = 0;
		}
		
		queue->ReadIndex = readIndex;

		queue->Status = CIRCULAR_QUEUE_STATUS_NOT_FULL;
		
		return CIRCULAR_QUEUE_ERROR_OK;
	}
}
