#include "sbus.h"
#include "time.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "usart_hal_config.h"

#define SBUS_DATA_LEN    32

uint8_t sbus_data_buff[SBUS_DATA_LEN] = {0};


IBus_handel_t ibus_handel = {0};
uint16_t ibus_solution_data[IBUS_Channel_MAX];



void sbus_init()
{
  GPIO_InitType GPIO_InitStructure;
  USART_InitType USART_InitStructure;
  NVIC_InitType NVIC_InitStructure;

  ibus_handel.data_read_index = 0;
  ibus_handel.data_read_flag = 0;


#if 0

  RCC_AHBPeriphClockCmd(SBUS_UART_RX_GPIO_RCC, ENABLE);

  RCC_APB1PeriphClockCmd(SBUS_UART_RCC, ENABLE); 

  GPIO_PinAFConfig(SBUS_UART_RX_GPIO, SBUS_UART_RX_GPIO_PINSSOURCE, SBUS_UART_RX_GPIO_AF);

  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;
  GPIO_InitStructure.GPIO_OutType = GPIO_OutType_PP;
  GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NOPULL;
  
  GPIO_InitStructure.GPIO_Pins = SBUS_UART_RX_GPIO_PIN;
  GPIO_Init(SBUS_UART_RX_GPIO, &GPIO_InitStructure);


  //USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = SBUS_UART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;     //USART_Parity_No  USART_Parity_Even
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx;

  USART_Init(SBUS_UARTx, &USART_InitStructure);
  
  USART_INTConfig(SBUS_UARTx, USART_INT_RDNE, ENABLE);
  //USART_INTConfig(UARTx, USART_INT_TDE, ENABLE);

  USART_Cmd(SBUS_UARTx, ENABLE);

  
  NVIC_InitStructure.NVIC_IRQChannel = SBUS_UART_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 14;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#endif
}



void sbus_updata_data()
{
  static uint8_t data_num = 0;
  uint8_t data;
  data = USART_ReceiveData(SBUS_UARTx);

  if((0 == data_num) && (0x0f == data))
  {
    sbus_data_buff[data_num] = data;
    data_num++;
  }
  else if((data_num > 0) && (data_num < 24))
  {
    sbus_data_buff[data_num] = data;
    data_num++;
  }
  else if((24 == data_num) && (0x00 == data))
  {
    sbus_data_buff[data_num] = data;
    data_num = 0;
  }
  else
  {
    data_num = 0;
  }


}


void ibus_set_data_buff(uint8_t index, uint8_t value)
{
  uint8_t write_array_index = ibus_handel.data_read_index ^ 0x01;
  ibus_handel.data[write_array_index].data_buff[index] = value;
}

uint8_t ibus_get_data_buff(uint8_t index)
{
  uint8_t read_array_index = ibus_handel.data_read_index;
  
  return ibus_handel.data[read_array_index].data_buff[index];
}



void ibus_add_data_buff_sum(uint8_t add_value)
{
  uint8_t write_array_index = ibus_handel.data_read_index ^ 0x01;
  ibus_handel.data[write_array_index].data_sum += add_value;
}

uint16_t ibus_get_data_buff_sum()
{
  uint8_t read_array_index = ibus_handel.data_read_index;
  return ibus_handel.data[read_array_index].data_sum;
}

void ibus_data_buff_clean()
{
  uint8_t write_array_index = ibus_handel.data_read_index ^ 0x01;
  ibus_handel.data[write_array_index].data_sum = 0;
}

uint8_t ibus_data_solution()
{
  int32_t i;
  uint8_t data_l;
  uint8_t data_h;
  uint16_t data_sum;
  uint16_t data_sum_check;


	data_sum = (((uint16_t)ibus_get_data_buff(31)) << 8) | ibus_get_data_buff(30);
  data_sum_check = ((ibus_get_data_buff_sum()+ 0x60) ^ 0xffff);
	  
  if(data_sum_check == data_sum)
  {
    for(i = 0;i < IBUS_Channel_MAX;i++)
    {
      data_l = ibus_get_data_buff((i << 1) + 2);
      data_h = ibus_get_data_buff((i << 1) + 3);
      ibus_solution_data[i] =  (((uint16_t)(data_h & 0x0f)) << 8) | data_l;
    }
    return 0;
  }   
  return 1;
}

uint16_t ibus_get_channel_value(IBUS_Channel_Type Channel)
{
  return ibus_solution_data[Channel];
}




//uint8_t ibus_time_flag = 0;
//uint32_t ibus_time = 0;


static int ibus_read_original_data(uint8_t data)
{
	int ret = 1;
  static uint8_t data_num = 0;

  //uint8_t data;
  extern xSemaphoreHandle remote_read_semaphore;

  //data = USART_ReceiveData(SBUS_UARTx);

  if((0 == data_num) && (0x20 == data))
  {
    /*if(0 == ibus_time_flag)
    {
      ibus_time_flag = 1;
      time_count_start_us();
    }
    else
    {
      ibus_time_flag = 0;
      ibus_time = time_count_end_us();
    }*/
    ibus_set_data_buff(data_num, data);
    data_num++;
  }
  else if((1 == data_num) && (0x40 == data))
  {
    ibus_set_data_buff(data_num, data);
    ibus_data_buff_clean();
    data_num++;
  }
  else if((data_num > 1) && (data_num < 30))    //30
  {
    ibus_set_data_buff(data_num, data);
	  ibus_add_data_buff_sum(data);
	  data_num++;
   
  }
  else if(30 == data_num)
  {
    ibus_set_data_buff(data_num, data);
    data_num++;
  }
  else if(31 == data_num)
  {
    ibus_set_data_buff(data_num, data);
    data_num = 0;
    ibus_handel.data_read_index = ibus_handel.data_read_index ^ 0x01;

	ret = 0;
    //xSemaphoreGiveFromISR(remote_read_semaphore,pdTRUE);
  }
  else
  {
    data_num = 0;
    ibus_handel.data_read_flag = 0;
  }

  	return ret;
}


int IBUS_Analysis()
{
	uint32_t i;
	int ret = 1;
	
	uint8_t data[32];
	uint32_t outLen = 0;
	
	USART_HAL_ReadData(&USART2_HAL, data, 32, &outLen);

	for(i = 0;i < outLen;++i)
	{
		if(0 == ibus_read_original_data(data[i]))
		{
			ret = 0;
		}
	}

	return ret;
}

