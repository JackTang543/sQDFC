#include "sBSP_UART.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "defines.h"

/**
 * sBSP_UART.c
 *
 */

/*Debug串口 UART1*/
UART_HandleTypeDef uart1;
static const uint32_t uart1_blocking_ms = 100;
// printf格式化
static char uart1_fmt_buf[512];
// 串口接收缓冲
static char uart1_recv_buf[512];
// 保存用户传入的接收完成回调
static sBSP_UART_RecvEndCb_t uart1_recv_end_cb;
// DMA
//  DMA_HandleTypeDef hdma_usart1_rx;
//  DMA_HandleTypeDef hdma_usart1_tx;

int sBSP_UART_Debug_Init(uint32_t bandrate) {
    uart1.Instance          = USART1;
    uart1.Init.BaudRate     = bandrate;
    uart1.Init.WordLength   = UART_WORDLENGTH_8B;
    uart1.Init.StopBits     = UART_STOPBITS_1;
    uart1.Init.Parity       = UART_PARITY_NONE;
    uart1.Init.Mode         = UART_MODE_TX_RX;
    uart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if(HAL_UART_Init(&uart1) != HAL_OK) {
        return -1;
    }
    return 0;
}

void sBSP_UART_Debug_Printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsprintf(uart1_fmt_buf, fmt, ap);
    va_end(ap);

    HAL_UART_Transmit(&uart1, (uint8_t*)uart1_fmt_buf, (uint16_t)(strlen(uart1_fmt_buf)), uart1_blocking_ms);
}

void sBSP_UART_Debug_SendByte(uint8_t byte) {
    HAL_UART_Transmit(&uart1, &byte, 1, uart1_blocking_ms);
}

void sBSP_UART_Debug_SendBytes(uint8_t* pData, uint16_t length) {
    HAL_UART_Transmit(&uart1, pData, length, uart1_blocking_ms);
}

void sBSP_UART_Debug_RecvBegin(sBSP_UART_RecvEndCb_t recv_cb) {
    assert_param(recv_cb != NULL);
    uart1_recv_end_cb = recv_cb;

    if(HAL_UARTEx_ReceiveToIdle_IT(&uart1, (uint8_t*)uart1_recv_buf, sizeof(uart1_recv_buf)) != HAL_OK) {
        sBSP_UART_Debug_Printf("串口1:空闲中断IT接收出错");
    }

    // if(HAL_UARTEx_ReceiveToIdle_DMA(&uart1, (uint8_t*)uart1_recv_buf, sizeof(uart1_recv_buf)) != HAL_OK) {
    //     sBSP_UART_Debug_Printf("串口1:空闲中断DMA接收出错");
    // }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if(huart->Instance == USART1) {
        assert_param(uart1_recv_end_cb != NULL);
        uart1_recv_end_cb(uart1_recv_buf, Size);
    }
}



void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef gpio = {0};
    if(huart->Instance == USART1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PB6     ------> USART1_TX
        PB7     ------> USART1_RX
        */
        __USART1_CLK_ENABLE();
        gpio.Pin       = DEBUG_RX_Pin | DEBUG_TX_Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &gpio);

    } else if(huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        gpio.Pin       = OP_TX_Pin | OP_RX_Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio);

    } else if(huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        gpio.Pin       = EX_TX_Pin | EX_RX_Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &gpio);

    } else if(huart->Instance == USART6) {
        __HAL_RCC_USART6_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**USART6 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        gpio.Pin       = GPS_TX_Pin | GPS_RX_Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &gpio);

    } else if(huart->Instance == UART4) {
        /* UART4 clock enable */
        __HAL_RCC_UART4_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**UART4 GPIO Configuration
        PA0-WKUP     ------> UART4_TX
        PA1     ------> UART4_RX
        */
        gpio.Pin       = DR_TX_Pin | DR_RX_Pin;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}
