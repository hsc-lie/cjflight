/**
  ******************************************************************************
  * File   : USART/Interrupt/at32f4xx_it.c
  * Version: V1.2.4
  * Date   : 2020-08-26
  * Brief  : Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "at32f4xx_it.h"

#include "led.h"
#include "time.h"
#include "tim_input_capture.h"
#include "pwm.h"
#include "uart.h"
#include "sbus.h"

#include "mpu6050.h"

#include "quaternion.h"



//extern void vPortSVCHandler();
//extern void xPortSysTickHandler();
//extern void xPortPendSVHandler();


/** @addtogroup AT32F421_StdPeriph_Examples
  * @{
  */


/** @addtogroup USART_Interrupt
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
/*void SVC_Handler(void)
{
	 vPortSVCHandler();
}*/

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
/*void PendSV_Handler(void)
{
	xPortPendSVHandler();
}*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*void SysTick_Handler(void)
{
	xPortSysTickHandler();
}*/

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_INT_RDNE) != RESET)
  {
    //uart_send_byte(USART1, USART_ReceiveData(USART1));
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
		//sbus_updata_data();
		ibus_read_original_data();
		
	}

	if(USART_GetITStatus(USART2, USART_INT_IDLEF) == SET)
	{
	
		USART_ReceiveData(USART2);
		//USART_ClearFlag();
		//USART_ClearITPendingBit(USART2, USART_INT_RDNE);
	}
}

int led_flag = 0;
int motor_count = 0;
void TMR6_GLOBAL_IRQHandler(void)
{
  if(TMR_GetINTStatus(TMR6, TMR_INT_Overflow) == SET)
  {
    TMR_ClearITPendingBit(TMR6, TMR_INT_Overflow);
      
    //tim_500ms_task();

    
	
  }

}

void TMR3_GLOBAL_IRQHandler(void)
{ 
  if(TMR_GetINTStatus(TMR3, TMR_INT_CC2) == SET) 
  {
    TMR_ClearITPendingBit(TMR3, TMR_INT_CC2);
    tim_get_ppm();
  }
}

void TMR15_GLOBAL_IRQHandler(void)
{ 
  if(TMR_GetINTStatus(TMR15, TMR_INT_CC1) == SET) 
  {
    TMR_ClearITPendingBit(TMR15, TMR_INT_CC1);
    tim_get_ppm();
  }
}


void EXTI1_0_IRQHandler(void)
{
  if(EXTI_GetIntStatus(EXTI_Line0) == SET)
  {
    EXTI_ClearIntPendingBit(EXTI_Line0);
	  tim_get_ppm();
  }
}



void DMA1_Channel7_4_IRQHandler(void)
{
	//static int i = 0;
	//static int j = 0;
	if(DMA_GetITStatus(DMA1_INT_HT5) == SET)
	{
		DMA_ClearITPendingBit(DMA1_INT_HT5);
		//i++;
	}
	
	if(DMA_GetITStatus(DMA1_INT_TC5) == SET)
	{
		DMA_ClearITPendingBit(DMA1_INT_TC5);
		//j++;
	}
}


/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2018 ArteryTek *****END OF FILE****/

