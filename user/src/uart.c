#include "uart.h"



uint8_t uart_send_byte_data = 0;
uint8_t uart_read_byte_data = 0;



void uart_init()
{
  GPIO_InitType GPIO_InitStructure;
  USART_InitType USART_InitStructure;
  NVIC_InitType NVIC_InitStructure;


  RCC_AHBPeriphClockCmd(UART_TX_GPIO_RCC | UART_RX_GPIO_RCC, ENABLE);

  
  RCC_APB2PeriphClockCmd(UART_RCC, ENABLE); 


  GPIO_PinAFConfig(UART_TX_GPIO, UART_TX_GPIO_PINSSOURCE, UART_TX_GPIO_AF);
  GPIO_PinAFConfig(UART_RX_GPIO, UART_RX_GPIO_PINSSOURCE, UART_RX_GPIO_AF);

  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;

  GPIO_InitStructure.GPIO_Pins = UART_TX_GPIO_PIN;
  GPIO_Init(UART_TX_GPIO, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pins = UART_RX_GPIO_PIN;
  GPIO_Init(UART_RX_GPIO, &GPIO_InitStructure);


  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = UART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(UARTx, &USART_InitStructure);
  
  USART_INTConfig(UARTx, USART_INT_RDNE, ENABLE);
  //USART_INTConfig(UARTx, USART_INT_TDE, ENABLE);

  USART_Cmd(UARTx, ENABLE);

  
  NVIC_InitStructure.NVIC_IRQChannel = UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}




void uart_send_byte(USART_Type * UART,uint8_t data)
{
    while(USART_GetFlagStatus(UART, USART_FLAG_TDE) == RESET);

    USART_SendData(UART, data);
}


int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
  uart_send_byte(UARTx, (uint8_t)ch);
	return (ch);
}


