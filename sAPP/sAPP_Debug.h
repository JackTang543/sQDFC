#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f4xx_hal.h"


void Error_Handler();
void Warning_Handler(uint8_t* file, uint32_t line);
void assert_failed(uint8_t* file, uint32_t line);

#ifdef __cplusplus
}
#endif

