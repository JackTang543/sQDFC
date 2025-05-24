#pragma once
#ifdef __cplusplus
extern "C"{
#endif



#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "sBSP_I2C.h"


void sDRV_AHT20_Init();

void sDRV_AHT20_StartMeasure();
void sDRV_AHT20_GetMeasure(float* temp,float* humi);



#ifdef __cplusplus
}
#endif

