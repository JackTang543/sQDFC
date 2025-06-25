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





int sBSP_TIM_DSHOT300_Init();
void sBSP_TIM_DSHOT300_SetEn(bool is_en);
void sBSP_TIM_DSHOT300_SendPacket(uint16_t lu1, uint16_t ru2, uint16_t ld3, uint16_t rd4);


#ifdef __cplusplus
}
#endif
