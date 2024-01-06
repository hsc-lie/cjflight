#include "ring_queue.h"

/*
 * @函数名 RingQueueInit
 * @说  明 环形队列初始化
 * @参  数 queue:环形队列
 *         buffer:环形队列缓存(该缓存大小要比需要大小多1字节,如需要8字节大小则传入的数组大小为8+1字节)
 *         bufferSize:环形队列缓存大小(同上需要加1,如需要8字节大小则传入8+1)
 * @返回值
*/
void RingQueueInit(RingQueue_t *queue, uint8_t *buffer, RingQueueBase_t bufferSize)
{
	queue->WriteIndex = 0;
	queue->ReadIndex = 0;
	queue->BufferSize = bufferSize;
	queue->Buffer = buffer;
}

/*
 * @函数名 RingQueueWriteByte
 * @说  明 环形队列写入一个字节
 * @参  数 queue:环形队列
 *         data:要写入的数据
 * @返回值 FALSE:队列已满未能写入
 *         TRUE:成功写入
*/
bool RingQueueWriteByte(RingQueue_t *queue, uint8_t data)
{
	RingQueueBase_t writeIndex = queue->WriteIndex;
	RingQueueBase_t writeIndexTemp = writeIndex;
	RingQueueBase_t readIndex = queue->ReadIndex;
	uint8_t *buffer = queue->Buffer;
	RingQueueBase_t bufferSize = queue->BufferSize;

	++writeIndex;
	if(writeIndex == bufferSize)
	{
		writeIndex = 0;
	}

	//队列已满
	if((writeIndex == readIndex) || (NULL == buffer))
	{
		return 0;
	}

	buffer[writeIndexTemp] = data;

	queue->WriteIndex = writeIndex;

	return TRUE;
}

/*
 * @函数名 RingQueueWriteData
 * @说  明 环形队列写入数据
 * @参  数 queue:环形队列
 *         data:要写入的数据
 *         len:数据长度
 * @返回值 0:队列已满
 *         其他:成功写入的大小 
*/
RingQueueBase_t RingQueueWriteData(RingQueue_t *queue, uint8_t *data, RingQueueBase_t len)
{
	RingQueueBase_t writeIndex = queue->WriteIndex;
	RingQueueBase_t writeIndeTemp = writeIndex;
	RingQueueBase_t writeEndIndex;
	RingQueueBase_t writtenSize = 0;
	RingQueueBase_t readIndex = queue->ReadIndex;
	uint8_t *buffer = queue->Buffer;
	RingQueueBase_t bufferSize = queue->BufferSize;
	

	if(NULL == buffer)
	{
		return 0;
	}

	if(writeIndex >= readIndex)
	{
		writeEndIndex = writeIndex + len;

		if(0u == readIndex)
		{
			--bufferSize;
		}

		if(writeEndIndex > bufferSize)
		{
			writeEndIndex = bufferSize;
		}

		for(;writeIndex < writeEndIndex;++writeIndex)
		{
			buffer[writeIndex] = *data++;
		}

		writtenSize = writeIndex - writeIndeTemp;

		if((0u == readIndex) || (writtenSize == len))
		{
			queue->WriteIndex = writeIndex;
			return writtenSize;
		}

		writeIndeTemp = writeIndex = 0;
	}

	writeEndIndex = writeIndex + len - writtenSize;

	//进入到这里时 readIndex!=0
	if(writeEndIndex > readIndex - 1)
	{
		writeEndIndex = readIndex - 1;
	}

	for(;writeIndex < writeEndIndex;++writeIndex)
	{
		buffer[writeIndex] = *data++;
	}

	writtenSize += writeIndex - writeIndeTemp;

	queue->WriteIndex = writeIndex;

	return writtenSize;
}

/*
 * @函数名 RingQueueReadByte
 * @说  明 环形队列读取一个字节
 * @参  数 queue:环形队列
 *         data:读取到的数据
 * @返回值 FALSE:队列为空
 *         TRUE:读取成功 
*/
bool RingQueueReadByte(RingQueue_t *queue, uint8_t *data)
{
	RingQueueBase_t readIndex = queue->ReadIndex;
	RingQueueBase_t writeIndex = queue->WriteIndex;
	uint8_t *buffer = queue->Buffer;
	RingQueueBase_t bufferSize = queue->BufferSize;
	
	//队列为空
	if((readIndex == writeIndex) || (NULL == buffer))
	{
		return FALSE;
	}

	*data = buffer[readIndex];

	++readIndex;
	if(readIndex == bufferSize)
	{
		readIndex = 0;
	}

	queue->ReadIndex = readIndex;

	return TRUE;
}

/*
 * @函数名 RingQueueReadData
 * @说  明 环形队列读取数据
 * @参  数 queue:环形队列
 *         data:读取到的数据
 *         len:需要读取数据的长度
 * @返回值 0:队列为空
 *         其他:成功读取到的大小 
*/
RingQueueBase_t RingQueueReadData(RingQueue_t *queue, uint8_t *data, RingQueueBase_t len)
{
	RingQueueBase_t readIndex = queue->ReadIndex;
	RingQueueBase_t readIndexTemp = readIndex;
	RingQueueBase_t readEndIndex;
	RingQueueBase_t readSize = 0;
	RingQueueBase_t writeIndex = queue->WriteIndex;
	uint8_t *buffer = queue->Buffer;
	RingQueueBase_t bufferSize = queue->BufferSize;
	
	if(NULL == buffer)
	{
		return 0;
	}

	if(readIndex > writeIndex)
	{
		readEndIndex = readIndex + len;

		if(readEndIndex > bufferSize)
		{
			readEndIndex = bufferSize;
		}

		for(;readIndex < readEndIndex;++readIndex)
		{
			*data++ = buffer[readIndex];
		}

		readSize = readIndex - readIndexTemp;
		
		if((readSize == len))
		{
			queue->ReadIndex = readIndex;
			return readSize;
		}

		readIndexTemp = readIndex = 0;
	}

	readEndIndex = readIndex + len - readSize;

	if(readEndIndex > writeIndex)
	{
		readEndIndex = writeIndex;
	}

	for(;readIndex < readEndIndex;++readIndex)
	{
		*data++ = buffer[readIndex];
	}

	readSize += readIndex - readIndexTemp;
	queue->ReadIndex = readIndex;

	return readSize;
}
