#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include "stm32f4xx_it.h"

int sBSP_TIM_IMUHeater_Init();
void sBSP_TIM_IMUHeater_SetPWMFreq(uint32_t freq);
void sBSP_TIM_IMUHeater_SetEN(bool is_en);
void sBSP_TIM_IMUHeater_SetDuty(float percent);



#ifdef __cplusplus
}
#endif
