#include "sBSP_SPI.h"

#include "defines.h"

// sBSP_SPI.c 20241009 v1
// used for sGCARC
// 20241120 v1.1 sGCARCv5

/**
 * sBSP_SPI.c 250518 v1
 * used for sQDFC
 *
 */

/*SPI1 -> IMU*/
SPI_HandleTypeDef hspi1;

/*SPI2 -> FeRAM*/
SPI_HandleTypeDef hspi2;

#include "sUtils.h"

/// @brief IMU SPI通信初始化
/// @param SPI_BAUDRATE SPI波特率分频系数,可选值为SPI_BAUDRATEPRESCALER_2 ~ SPI_BAUDRATEPRESCALER_256
/// @return 0:成功, -1:失败
int sBSP_SPI_IMU_Init(uint32_t SPI_BAUDRATE) {
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATE;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if(HAL_SPI_Init(&hspi1) != HAL_OK) {
        return -1;
    }

    return 0;
}



void sBSP_SPI_IMU_SetEN(uint8_t en) {
    en ? __HAL_SPI_ENABLE(&hspi1) : __HAL_SPI_DISABLE(&hspi1);
}

void sBSP_SPI_IMU_SendByte(uint8_t byte) {
    HAL_SPI_Transmit(&hspi1, &byte, 1, 100);
}

uint8_t sBSP_SPI_IMU_RecvByte() {
    uint8_t send_byte = 0;
    HAL_SPI_Receive(&hspi1, &send_byte, 1, 100);
    return send_byte;
}

void sBSP_SPI_IMU_SendBytes(uint8_t* pData, uint16_t Size) {
    HAL_SPI_Transmit(&hspi1, pData, Size, 1000);
}

void sBSP_SPI_IMU_RecvBytes(uint8_t* pData, uint16_t Size) {
    HAL_SPI_Receive(&hspi1, pData, Size, 1000);
}

// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {

//     if(hspi->Instance == SPI1) {

//         // 恢复CS默认高电平(用于DMA异步处理)
//         //  sBSP_SPI_OLED_SetCS(1);
//     }
// }

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(spiHandle->Instance == SPI1) {
        /* SPI1 clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin       = IMU_SCK_Pin | IMU_MISO_Pin | IMU_MOSI_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // HAL_NVIC_SetPriority(SPI1_IRQn, 3, 0);
        // HAL_NVIC_EnableIRQ(SPI1_IRQn);

    } else if(spiHandle->Instance == SPI2) {
        /* SPI2 clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**SPI2 GPIO Configuration
        PC2     ------> SPI2_MISO
        PC3     ------> SPI2_MOSI
        PB13     ------> SPI2_SCK
        */
        GPIO_InitStruct.Pin       = FERAM_MISO_Pin | FERAM_MOSI_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = FERAM_SCK_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(FERAM_SCK_GPIO_Port, &GPIO_InitStruct);
    }
}
