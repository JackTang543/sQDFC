#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include "stm32f4xx_it.h"

extern SPI_HandleTypeDef hspi1;

/*IMU SPI interface*/
int sBSP_SPI_IMU_Init(uint32_t SPI_BAUDRATE);
void sBSP_SPI_IMU_SetEN(uint8_t en);
void sBSP_SPI_IMU_SendByte(uint8_t byte);
uint8_t sBSP_SPI_IMU_RecvByte();
void sBSP_SPI_IMU_SendBytes(uint8_t* pData, uint16_t Size);
void sBSP_SPI_IMU_RecvBytes(uint8_t* pData, uint16_t Size);

/*FeRAM SPI interface*/
int sBSP_SPI_FERAM_Init(uint32_t SPI_BAUDRATE);
void sBSP_SPI_FERAM_SetEN(uint8_t en);
void sBSP_SPI_FERAM_SendByte(uint8_t byte);
uint8_t sBSP_SPI_FERAM_RecvByte();
void sBSP_SPI_FERAM_SendBytes(uint8_t* pData, uint16_t Size);
void sBSP_SPI_FERAM_RecvBytes(uint8_t* pData, uint16_t Size);

#ifdef __cplusplus
}
#endif
