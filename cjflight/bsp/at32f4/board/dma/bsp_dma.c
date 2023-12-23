#include "bsp_dma.h"
#include "bsp_usart.h"
#include "at32f4xx.h"




#define USART2_RX_DMA_BUFFER_SIZE        (32)
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
	uint32_t i;
	uint32_t halfSize = USART2_RX_DMA_BUFFER_SIZE >> 1;

	for(i = 0;i < halfSize;++i)
	{
		CircularQueue_WriteByte(&USART2Queue, USARTRxDMABuffer[i]);
	}
		
}


void DMA1_CH5_TC_Transmission()
{
	uint32_t i;
	uint32_t halfSize = USART2_RX_DMA_BUFFER_SIZE >> 1;

	for(i = halfSize;i < USART2_RX_DMA_BUFFER_SIZE;++i)
	{
		CircularQueue_WriteByte(&USART2Queue, USARTRxDMABuffer[i]);
	}
}

