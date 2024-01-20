#include "bsp_dma.h"
#include "bsp_usart.h"
#include "bsp_timer.h"
#include "ring_queue.h"


#define USART2_RX_DMA_BUFFER_SIZE        (64)
static uint8_t USARTRxDMABuffer[USART2_RX_DMA_BUFFER_SIZE];


void BSPDMA1Cannel5Init()
{
	DMA_InitType dmaInitStruct;
	NVIC_InitType nvicInitStructure;
 
  	dmaInitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DT;
  	dmaInitStruct.DMA_MemoryBaseAddr = (uint32_t)&USARTRxDMABuffer;
  	dmaInitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
  	dmaInitStruct.DMA_BufferSize = USART2_RX_DMA_BUFFER_SIZE;
	dmaInitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
	dmaInitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
	dmaInitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
	dmaInitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
	dmaInitStruct.DMA_Mode = DMA_MODE_CIRCULAR;
	dmaInitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
	dmaInitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

	
  	DMA_Init(DMA1_Channel5, &dmaInitStruct);

	DMA_INTConfig(DMA1_Channel5, DMA_INT_HT, ENABLE);
	DMA_INTConfig(DMA1_Channel5, DMA_INT_TC, ENABLE);
	

	DMA_ChannelEnable(DMA1_Channel5, ENABLE);


	nvicInitStructure.NVIC_IRQChannel = DMA1_Channel7_4_IRQn;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInitStructure);
}



void DMA1_CH5_HT_Transmission()
{
	uint32_t halfSize = USART2_RX_DMA_BUFFER_SIZE >> 1;
#if 1
	RingQueueWriteData(&USART2RingQueue, USARTRxDMABuffer, halfSize);	
#else
	uint8_t *p;
	uint8_t *pEnd = USARTRxDMABuffer + halfSize;
	for(p = USARTRxDMABuffer;p < pEnd;p++)
	{
		RingQueueWriteByte(&USART2RingQueue, *p);
	}
#endif
}


void DMA1_CH5_TC_Transmission()
{
	uint32_t halfSize = USART2_RX_DMA_BUFFER_SIZE >> 1;
#if 1
	RingQueueWriteData(&USART2RingQueue, USARTRxDMABuffer+halfSize, halfSize);
#else
	uint8_t *p;
	uint8_t *pEnd = USARTRxDMABuffer + USART2_RX_DMA_BUFFER_SIZE;
	for(p = USARTRxDMABuffer + halfSize;p < pEnd;p++)
	{
		RingQueueWriteByte(&USART2RingQueue, *p);
	}
#endif
}


void DMA1_Channel7_4_IRQHandler(void)
{
	//static int i = 0;
	//static int j = 0;
	
	//传输一半中断
	if(DMA_GetITStatus(DMA1_INT_HT5) == SET)
	{
		DMA_ClearITPendingBit(DMA1_INT_HT5);
		DMA1_CH5_HT_Transmission();	
		//++i;
	}
	//传输完成中断
	if(DMA_GetITStatus(DMA1_INT_TC5) == SET)
	{
		DMA_ClearITPendingBit(DMA1_INT_TC5);
		DMA1_CH5_TC_Transmission();	
		//++j;
	}
}
