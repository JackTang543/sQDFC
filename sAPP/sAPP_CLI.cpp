#include "sAPP_CLI.hpp"

#include "FreeRTOS.h"
#include "semphr.h"

#include "sAPP_AHRS.hpp"
#include "sAPP_Tasks.hpp"

#include "sBSP_UART.h"

#include "cstring"

#include "FreeRTOS-Plus-CLI\FreeRTOS_CLI.h"

enum class SEND_DATA_TYPE {
    NONE  = 0,
    EULER = 1,
};

typedef struct {
    SEND_DATA_TYPE type;
    uint16_t send_intervals_ms;
    bool send_en;
} send_data_pactet_t;

// 串口接收到数据信号量
static SemaphoreHandle_t sem_debug_uart_recved;
static send_data_pactet_t cli_send_data_packet;

static char* uart_recv_buf;
static uint16_t uart_recv_len;

static void uart_recved(char* pReciData, uint16_t length) {
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    uart_recv_buf = pReciData;
    uart_recv_len = length;

    xSemaphoreGiveFromISR(sem_debug_uart_recved, &xHigherPriorityTaskWoken);

    sBSP_UART_Debug_RecvBegin(uart_recved);

    // 请求上下文切换，如果有更高优先级的任务被唤醒
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*软件复位系统*/
static BaseType_t reset(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString) {
    // 不做操作,因为MCU马上就要复位了
    NVIC_SystemReset();
    return pdFALSE; // 表示不需要更多的输出 命令已处理完毕
}
const CLI_Command_Definition_t comm_reset = {
    "reset",                         // 命令的文本
    "reset:复位所有系统并重启MCU\n", // 命令的帮助文本
    reset,                           // 命令的处理函数
    0                                // 参数的数量
};

/*实时打印欧拉角*/
static BaseType_t showEuler(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString) {

    cli_send_data_packet.send_en           = 1;
    cli_send_data_packet.send_intervals_ms = 20;
    cli_send_data_packet.type              = SEND_DATA_TYPE::EULER;

    return pdFALSE;
}
const CLI_Command_Definition_t comm_showEuler = {
    "showEuler",                               
    "showEuler:显示欧拉角数据流(50Hz),输入stop停止\n", 
    showEuler,                                 
    0                                       
};

/*手动校准陀螺仪零偏*/
static BaseType_t calibGyrBias(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString) {
    sAPP_Tasks_StartCalibGyrBias();
    return pdFALSE;
}
const CLI_Command_Definition_t comm_calibGyrBias = {
    "calibGyrBias",                               
    "calibGyrBias:手动校准陀螺仪静态零偏\n", 
    calibGyrBias,                                 
    0                                       
};


/*停止任何的循环打印数据流*/
static BaseType_t stop(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString) {
    if(cli_send_data_packet.send_en == 1) {
        cli_send_data_packet.send_en = 0;
        strncpy(pcWriteBuffer, "stop:已停止打印数据流\n", xWriteBufferLen);
    } else {
        strncpy(pcWriteBuffer, "stop:数据流已在停止状态\n", xWriteBufferLen);
    }

    return pdFALSE;
}
const CLI_Command_Definition_t comm_stop = {
    "stop",
    "stop:停止任何的循环打印数据流\n",
    stop,
    0,
};




int sAPP_CLI_Init() {
    sem_debug_uart_recved = xSemaphoreCreateBinary();

    FreeRTOS_CLIRegisterCommand(&comm_reset);
    FreeRTOS_CLIRegisterCommand(&comm_showEuler);
    FreeRTOS_CLIRegisterCommand(&comm_calibGyrBias);




    FreeRTOS_CLIRegisterCommand(&comm_stop);

    sBSP_UART_Debug_RecvBegin(uart_recved);
    

    return 0;
}

void sAPP_CLI_RecvTask(void* param) {
    char* p_str_buf;
    BaseType_t more_data_flag;

    for(;;) {
        // 如果串口收到数据了
        if(xSemaphoreTake(sem_debug_uart_recved, portMAX_DELAY) == pdTRUE) {

            // 处理接收到的每一条命令
            do {
                // 调用 FreeRTOS+CLI 的命令处理器
                more_data_flag = FreeRTOS_CLIProcessCommand(
                    (const char*)uart_recv_buf, // 命令字符串
                    cOutputBuffer,              // 命令输出缓冲区
                    sizeof(cOutputBuffer)       // 缓冲区大小
                );

                // 将命令处理的输出发送回UART
                sBSP_UART_Debug_SendBytes((uint8_t*)cOutputBuffer, strlen(cOutputBuffer));


                //清空接收缓冲区
                memset(uart_recv_buf, 0, 512);

            } while(more_data_flag != pdFALSE); // 如果还有更多输出，继续处理
        }
    }
}

void sAPP_CLI_SendTask(void* param){

    for(;;){
        if(cli_send_data_packet.send_en == 1){
            //发送欧拉角
            if(cli_send_data_packet.type == SEND_DATA_TYPE::EULER){
                //先获取锁
                float pitch, roll, yaw;
                float imu_temp;
                if(xSemaphoreTake(ahrs.output.lock, 200) == pdTRUE){
                    //获取欧拉角
                    pitch = ahrs.output.pitch;
                    roll  = ahrs.output.roll;
                    yaw   = ahrs.output.yaw;
                    imu_temp = ahrs.raw_data.imu_temp;
                    //释放锁
                    xSemaphoreGive(ahrs.output.lock);
                }
                sBSP_UART_Debug_Printf("%.2f,%.2f,%.2f,%.1f\n", pitch, roll, yaw, imu_temp);
            }

            vTaskDelay(cli_send_data_packet.send_intervals_ms / portTICK_PERIOD_MS);
        }else{
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

