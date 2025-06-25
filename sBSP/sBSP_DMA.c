#include "sBSP_DMA.h"

void sBSP_DMA_Init() {
    // __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // /* DMA interrupt init */
    // /* DMA2_Stream1_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    // /* DMA2_Stream2_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    // /* DMA2_Stream4_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    // /* DMA2_Stream6_IRQn interrupt configuration */
    // HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

    /* DMA interrupt init */
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    /* DMA2_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}
