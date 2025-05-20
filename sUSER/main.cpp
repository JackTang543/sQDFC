#include "main.hpp"

#include "stm32f4xx_hal.h"
#include "defines.h"

#include "sUtils.h"
#include "sBSP_UART.h"

/**
 * todo list
 * 1. 移植Arduino Framework
 * 2. 移植FreeRTOS
 * ? 3. 移植HAL库 完成
 * 4. cm_backtrace
 * 5. sBinOutDrv
 * 6. sDWTLib
 * 7. 驱动SD卡 SDIO
 * 8. 驱动IMU ICM45686
 * 9. 驱动IMU LIS3MDL
 * 10. 驱动MB85RS256
 * 11. 驱动WS2812
 * !12. AHT20
 * !13. SPL01
 * 14. ADC
 * !15. IMU调温
 * 16. 打通MAVLink通信数据链
 * ? 17. 移植ekf_AltEst6
 * 18. 移植sLib的Fliter
 * 19. 移植FreeRTOS的CLI
 *
 *  */

int main() {
    HAL_Init(); // 初始化HAL库

    sBSP_RCC_Init();

    sBSP_UART_Debug_Init(115200);

    //启用除0异常
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    //初始化cm_backtrace崩溃调试
    cm_backtrace_init(APPNAME, HARDWARE_VERSION, SOFTWARE_VERSION);

    sBSP_TIM_IMUHeater_Init();
    sBSP_TIM_IMUHeater_SetPWMFreq(84000);
    // sBSP_TIM_IMUHeater_SetDuty(0.5f);
    sBSP_TIM_IMUHeater_SetEN(1);

    sAPP_CLI_Init();


    log_printfln("Hello I'm sightseer's Quad Drone Flight Controller v1"); // 打印Hello World!

    

    HAL_Delay(100);

    // 配置RUN LED引脚为输出
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = RUN_LED_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RUN_LED_GPIO_Port, &GPIO_InitStruct);

    ahrs.init(AHRS::IMUType::ICM45686, AHRS::MAGType::LIS3MDLTR); // 初始化AHRS




    sBSP_UART_Debug_Printf("Current free heap size: %u bytes\n", (unsigned int)xPortGetFreeHeapSize());

    sAPP_Tasks_CreateAll();
    sBSP_UART_Debug_Printf("Current free heap size: %u bytes\n", (unsigned int)xPortGetFreeHeapSize());
    sBSP_UART_Debug_Printf("FreeRTOS启动任务调度\n");
    vTaskStartScheduler();

    while(1) {
        // SCK toggle
        //  HAL_GPIO_TogglePin(IMU_SCK_GPIO_Port,IMU_SCK_Pin); //翻转SCK引脚
        //  HAL_GPIO_TogglePin(IMU_MISO_GPIO_Port,IMU_MOSI_Pin); //翻转MISO引脚
        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin); // 翻转RUN LED引脚
        HAL_Delay(100);                                     // 延时500ms
    }
}
