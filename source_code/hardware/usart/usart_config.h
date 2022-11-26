#ifndef __USART_CONFIG_H_
#define __USART_CONFIG_H_


#include "common.h"
#include "circular_queue.h"

#define USART2_BUFFER_SIZE        (64)


extern CircularQueue_t USART2_Queue;


extern void USART_ConfigInitAll(void);




#endif /*__USART_CONFIG_H_*/
