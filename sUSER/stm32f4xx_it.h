#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);


void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);



#ifdef __cplusplus
}
#endif

#endif
