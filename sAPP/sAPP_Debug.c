#include "sAPP_Debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include "sUtils.h"




void Error_Handler(){
    __disable_irq();
    while (1){
        log_error("发生错误: 禁用IRQ,死循环...");
        HAL_Delay(500);
    }
}

void Warning_Handler(uint8_t* file, uint32_t line){
    log_warn("文件名:%s,行:%u",file,line);
}




void assert_failed(uint8_t* file, uint32_t line){
    __disable_irq();
    while (1){
        log_assert("断言失败! 文件名:%s,行:%u 禁用IRQ,死循环...",file,line);
        HAL_Delay(500);
    }
}

void vApplicationMallocFailedHook(){
    log_warn("警告:内存申请失败!");
}

void vApplicationIdleHook(){
    
}   

void vApplicationTickHook(){
}


void vApplicationStackOverflowHook(TaskHandle_t xTask,char* pcTaskName){
    log_warn("警告:栈溢出! 任务句柄:0x%X,任务名:%s",xTask,pcTaskName);
}


