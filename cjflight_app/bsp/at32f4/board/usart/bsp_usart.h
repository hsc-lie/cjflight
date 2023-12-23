#ifndef __BSP_USART_H_
#define __BSP_USART_H_

#include "at32f4xx.h"

#include "common.h"
#include "circular_queue.h"

#define USART2_BUFFER_SIZE        (64)


extern CircularQueue_t USART2Queue;


extern void BSPUSART1Init(void);
extern void BSPUSART2Init(void);
extern void BSPUSARTSendData(USART_Type * USARTx,  uint8_t * data, uint32_t len);


#endif /*__BSP_USART_H_*/
