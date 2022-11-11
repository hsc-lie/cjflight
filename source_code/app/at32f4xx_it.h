/**
  ******************************************************************************
  * File   : USART/Interrupt/at32f4xx_it.h
  * Version: V1.2.4
  * Date   : 2020-08-26
  * Brief  : Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT32F4XX_IT_H
#define __AT32F4XX_IT_H

/* Includes ------------------------------------------------------------------*/
#include "at32f4xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#endif /* __AT32F4XX_IT_H */

/******************* (C) COPYRIGHT 2018 ArteryTek *****END OF FILE****/