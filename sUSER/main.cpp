#include "main.hpp"

#include "stm32f4xx_hal.h"
#include "defines.h"

#include "sUtils.h"
#include "sBSP_UART.h"


/**
 * todo list
 * 1. 移植Arduino Framework
 * 2. 移植FreeRTOS
 * 3. 移植HAL库
 * 4. cm_backtrace
 * 5. sBinOutDrv
 * 6. sDWTLib
 * 7. 驱动SD卡 SDIO
 * 8. 驱动IMU ICM45686
 * 9. 驱动IMU LIS3MDL
 * 10. 驱动MB85RS256
 * 11. 驱动WS2812
 * 12. AHT20
 * 13. SPL01
 * 14. ADC
 * 15. IMU调温
 * 16. 打通MAVLink通信数据链
 * 17. 移植ekf_AltEst6
 * 
 * 
 *  */

int main(){
    HAL_Init(); //初始化HAL库

    sBSP_UART_Debug_Init(115200); //初始化串口1
    
    log_printfln("Hello I'm sightseer's Quad Drone Flight Controller v1"); //打印Hello World!


    //配置RUN LED引脚为输出
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RUN_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RUN_LED_GPIO_Port, &GPIO_InitStruct);
    


    while(1){
        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port,RUN_LED_Pin); //翻转RUN LED引脚
        HAL_Delay(500); //延时500ms
        
    }
}











