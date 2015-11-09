/**
  ******************************************************************************
  * @file    NVIC/NVIC_WFIMode/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    04-September-2015
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 
#include "stm32f4xx.h"
#include "stm324xg_eval.h"

void USART3_IRQHandler(void);
void UART_CMD_HANDLER(void);
#define SIZE_BUF 255
uint8_t Buffercmp8(uint8_t *pBuffer1, uint8_t *pBuffer2, uint8_t BufferLength);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
