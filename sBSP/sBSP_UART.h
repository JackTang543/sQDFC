#pragma once
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
// #include "sDBG_Debug.h"
#include <stdbool.h>




typedef void(*sBSP_UART_RecvEndCb_t)(char* pReciData,uint16_t length);




int sBSP_UART_Debug_Init(uint32_t bandrate);
void sBSP_UART_Debug_RecvBegin(sBSP_UART_RecvEndCb_t recv_cb);
void sBSP_UART_Debug_Printf(const char *fmt,...);
void sBSP_UART_Debug_SendBytes(uint8_t* pData,uint16_t length);

int sBSP_UART_OpticalFlow_Init(uint32_t bandrate);
int sBSP_UART_OpticalFlow_SendBytes(uint8_t* pData,uint16_t length);
void sBSP_UART_OpticalFlow_RecvBegin(sBSP_UART_RecvEndCb_t recv_cb);
void sBSP_UART_OpticalFlow_RecvContinue();


void sBSP_UART_IMU_Printf(const char *fmt,...);
void sBSP_UART_IMU_RecvBegin(sBSP_UART_RecvEndCb_t recv_cb);

int sBSP_UART_Top_Init(uint32_t bandrate);
void sBSP_UART_Top_Printf(const char *fmt,...);
void sBSP_UART_Top_RecvBegin(sBSP_UART_RecvEndCb_t recv_cb);



#ifdef __cplusplus
}
#endif

