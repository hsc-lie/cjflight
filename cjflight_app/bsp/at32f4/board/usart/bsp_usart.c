#include "bsp_usart.h"

#include "at32f4xx_usart.h"

            
static uint8_t USART2Buffer[USART2_BUFFER_SIZE+1];
RingQueue_t USART2RingQueue;


void BSPUSART1Init()
{
	USART_InitType usartInitStructure;
	//NVIC_InitType nvicInitStructure;

	RingQueueInit(&USART2RingQueue, USART2Buffer, USART2_BUFFER_SIZE+1);

	USART_StructInit(&usartInitStructure);
	usartInitStructure.USART_BaudRate = 115200;
	usartInitStructure.USART_WordLength = USART_WordLength_8b;
	usartInitStructure.USART_StopBits = USART_StopBits_1;
	usartInitStructure.USART_Parity = USART_Parity_No;
	usartInitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartInitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &usartInitStructure);

	//USART_INTConfig(USART1, USART_INT_RDNE, ENABLE);
	//USART_INTConfig(USART1, USART_INT_TDE, ENABLE);

	USART_Cmd(USART1, ENABLE);
	/*
	nvicInitStructure.NVIC_IRQChannel = USART1_IRQn;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInitStructure);*/
}




void BSPUSART2Init()
{
	USART_InitType usartInitStructure;
	NVIC_InitType nvicInitStructure;

	usartInitStructure.USART_BaudRate = 115200;
	usartInitStructure.USART_WordLength = USART_WordLength_8b;
	usartInitStructure.USART_StopBits = USART_StopBits_1;
	usartInitStructure.USART_Parity = USART_Parity_No;     //USART_Parity_No  USART_Parity_Even
	usartInitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartInitStructure.USART_Mode = USART_Mode_Rx;

  	USART_Init(USART2, &usartInitStructure);
  
  	//USART_INTConfig(USART2, USART_INT_RDNE, ENABLE);
	//USART_INTConfig(USART2, USART_INT_IDLEF, ENABLE);
  	//USART_INTConfig(UARTx, USART_INT_TDE, ENABLE);

  	USART_Cmd(USART2, ENABLE);
	
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
  
	nvicInitStructure.NVIC_IRQChannel = USART2_IRQn;
	nvicInitStructure.NVIC_IRQChannelPreemptionPriority = 14;
	nvicInitStructure.NVIC_IRQChannelSubPriority = 0;
	nvicInitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInitStructure);

}


void BSPUSARTSendData(USART_Type * USARTx,  uint8_t * data, uint32_t len)
{
	uint32_t i;
	for(i = 0;i < len;i++)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TDE) == RESET);
    	USART_SendData(USARTx, *data);
		++data;
	}
}

uint32_t BSPUSART2ReadData(uint8_t *data, uint32_t len)
{
	return RingQueueReadData(&USART2RingQueue, data, len);
	//return 0;
}

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_INT_RDNE) != RESET)
	{

	}
}

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_INT_RDNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_INT_RDNE);	
	}

	if(USART_GetITStatus(USART2, USART_INT_IDLEF) == SET)
	{
    	USART_ClearITPendingBit(USART2, USART_INT_IDLEF);	
	}
}
