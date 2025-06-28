#include "sAPP_OpticalFlow.hpp"

#include "sUtils.h"

#include "sDRV_MTF01P.hpp"

#include "sBSP_UART.h"
#include "defines.h"

#include "semphr.h"
#include "task.h"

sDRV_MTF01P::result_t optical_result = {0};

static SemaphoreHandle_t optical_flow_data_ready;

void sAPP_OF_DataReadyCbISR(char* pReciData, uint16_t length);

void sAPP_OF_Task(void* param) {
    log_info("光流传感器任务开始运行");

    optical_flow_data_ready = xSemaphoreCreateBinary();

    optical_flow.init(); // 初始化光流传感器

    sBSP_UART_OpticalFlow_RecvBegin(sAPP_OF_DataReadyCbISR); // 开始接收光流数据,并设置数据就绪回调

    for(;;) {
        if(xSemaphoreTake(optical_flow_data_ready, portMAX_DELAY) == pdTRUE) {
            // 获取光流传感器数据
            if(optical_flow.getResult(&optical_result) == 0) {
                // 成功获取数据
                log_printfln("X轴:%.2f m/s, Y轴:%.2f m/s, 激光距离:%.2f m, 光流置信度:%.2f, 激光强度:%.2f, 距离精度:%u, 激光有效:%d, 光流有效:%d",
                             optical_result.x_velo,
                             optical_result.y_velo,
                             optical_result.laser_dis,
                             optical_result.optical_quality,
                             optical_result.laser_strength,
                             optical_result.distance_accuracy,
                             optical_result.is_laser_valid,
                             optical_result.is_optical_valid);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // 延时10ms,避免过于频繁的获取数据
    }
}

void IRAM1_ATTR sAPP_OF_DataReadyCbISR(char* pReciData, uint16_t length) {
    optical_flow.recv_data_cb((uint8_t*)pReciData, length);

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(optical_flow_data_ready, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // 如果需要,让调度器立即切换到更高优先级的任务
    sBSP_UART_OpticalFlow_RecvContinue();         // 继续接收数据
}
