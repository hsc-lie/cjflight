#ifndef __DMA_CONFIG_H_
#define __DMA_CONFIG_H_


#include "common.h"

#define DMA_USART2_RX_BUFFER_SIZE        (32)


extern uint8_t DMA_USARTRXBuffer[DMA_USART2_RX_BUFFER_SIZE];


extern void DMA_ConfigInitAll(void);

extern void DMA1_CH5_HT_Transmission(void);
extern void DMA1_CH5_TC_Transmission(void);



#endif /*__DMA_CONFIG_H_*/
