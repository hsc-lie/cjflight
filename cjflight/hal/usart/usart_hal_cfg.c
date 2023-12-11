#include "usart_hal_cfg.h"

#include "bsp_usart.h"
#include "circular_queue.h"



//void (*SendData)(uint8_t data, uint32_t len);
//void (*ReadData)(uint8_t * data, uint32_t readLen, uint32_t * outLen);


static void USART1_HAL_SendData(uint8_t * data, uint32_t len)
{
	BSPUSARTSendData(USART1, data, len);
}



USART_HAL_t USART1_HAL = 
{
	.SendData = USART1_HAL_SendData,
	.ReadData = NULL,
};




static void USART2_HAL_ReadData(uint8_t * data, uint32_t readLen, uint32_t * outLen)
{
	uint32_t i;
	
	for(i = 0;i < readLen;++i)
	{
	
		if(E_CIRCULAR_QUEUE_ERROR_OK != CircularQueue_ReadByte(&USART2Queue, data))
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








