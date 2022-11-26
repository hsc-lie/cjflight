#include "dma_config.h"

#include "at32f4xx.h"


#include "usart_config.h"


uint8_t DMA_USARTRXBuffer[DMA_USART2_RX_BUFFER_SIZE];


void DMA_ConfigInitAll()
{
	DMA_InitType DMA_InitStruct;
	NVIC_InitType NVIC_InitStructure;
 
  	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DT;
  	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&DMA_USARTRXBuffer;
  	DMA_InitStruct.DMA_Direction = DMA_DIR_PERIPHERALSRC;
  	DMA_InitStruct.DMA_BufferSize = DMA_USART2_RX_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
	DMA_InitStruct.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
	DMA_InitStruct.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
	DMA_InitStruct.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
	DMA_InitStruct.DMA_Mode = DMA_MODE_CIRCULAR;
	DMA_InitStruct.DMA_Priority = DMA_PRIORITY_VERYHIGH;
	DMA_InitStruct.DMA_MTOM = DMA_MEMTOMEM_DISABLE;

	
  	DMA_Init(DMA1_Channel5, &DMA_InitStruct);

	DMA_INTConfig(DMA1_Channel5, DMA_INT_HT, ENABLE);
	DMA_INTConfig(DMA1_Channel5, DMA_INT_TC, ENABLE);
	

	DMA_ChannelEnable(DMA1_Channel5, ENABLE);


	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



void DMA1_CH5_HT_Transmission()
{
	uint32_t i;
	uint32_t halfSize = DMA_USART2_RX_BUFFER_SIZE >> 1;

	for(i = 0;i < halfSize;++i)
	{
		CircularQueue_WriteByte(&USART2_Queue, DMA_USARTRXBuffer[i]);
	}
		
}


void DMA1_CH5_TC_Transmission()
{
	uint32_t i;
	uint32_t halfSize = DMA_USART2_RX_BUFFER_SIZE >> 1;

	for(i = halfSize;i < DMA_USART2_RX_BUFFER_SIZE;++i)
	{
		CircularQueue_WriteByte(&USART2_Queue, DMA_USARTRXBuffer[i]);
	}
}

