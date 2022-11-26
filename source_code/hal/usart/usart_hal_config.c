#include "usart_hal_config.h"

#include "usart_config.h"
#include "circular_queue.h"



void (*SendData)(uint8_t data, uint32_t len);
void (*ReadData)(uint8_t * data, uint32_t readLen, uint32_t * outLen);


static void USART2_HAL_ReadData(uint8_t * data, uint32_t readLen, uint32_t * outLen)
{
	uint32_t i;
	
	for(i = 0;i < readLen;++i)
	{
	
		if(E_CIRCULAR_QUEUE_ERROR_OK != CircularQueue_ReadByte(&USART2_Queue, data))
		{
			break;
		}
		else
		{
			++data;
		}	
		
	}

	*outLen = i;
}


USART_HAL_t USART2_HAL = 
{
	.SendData = NULL,
	.ReadData = USART2_HAL_ReadData,
};








