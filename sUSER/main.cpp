#include "main.hpp"

#include "stm32f4xx_hal.h"
#include "defines.h"

#include "sUtils.h"
#include "sBSP_UART.h"

/**
 * todo list
 * ? 1. 移植Arduino Framework
 * ? 2. 移植FreeRTOS
 * ? 3. 移植HAL库 完成
 * ? 4. cm_backtrace
 * 5. sBinOutDrv
 * ? 6. sDWTLib
 * 7. 驱动SD卡 SDIO
 * ? 8. 驱动IMU ICM45686
 * ? 9. 驱动IMU LIS3MDL
 * ? 10. 驱动MB85RC64
 * 11. 驱动WS2812
 * ? 12. AHT20
 * !13. SPL01
 * 14. ADC
 * ? 15. IMU调温
 * 16. 打通MAVLink通信数据链
 * ? 17. 移植ekf_AltEst6
 * ? 18. 移植sLib的Fliter
 * ? 19. 移植FreeRTOS的CLI
 * 20. failsafe1,BKIN检测,最快速度保护
 * 21. failsafe2,pitch/roll>20度保护
 * 22. failsafe3,遥控器信号丢失保护
 * 23. failsafe4,电池电压过低保护
 * 24. failsafe5,各种异常自动保护
 * 25. failsafe6,惯导异常保护
 * 26. 写MTF-01P的驱动
 *
 *  */

int main() {
    HAL_Init(); // 初始化HAL库

    HAL_Delay(1000);

    sBSP_RCC_Init();

    // sBSP_UART_Debug_Init(115200);
    // sBSP_UART_Debug_Init(115200);
    sBSP_UART_Debug_Init(2'000'000);


    // 启用除0异常
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    // 初始化cm_backtrace崩溃调试
    cm_backtrace_init(APPNAME, HARDWARE_VERSION, SOFTWARE_VERSION);

    dwt.init(HAL_RCC_GetSysClockFreq());

    // log_printfln("SYS CLK = %u Hz", HAL_RCC_GetSysClockFreq()); // 打印系统时钟频率

    // HAL_Delay(100);

    sBSP_DMA_Init(); // 初始化DMA

    // sBSP_TIM_IMUHeater_Init();
    // sBSP_TIM_IMUHeater_SetPWMFreq(50000); // 50KHz
    // sBSP_TIM_IMUHeater_SetDuty(0.5f);
    // sBSP_TIM_IMUHeater_SetEN(0); // 关闭IMU加热器

    if(sBSP_TIM_DSHOT300_Init() != 0) {
        sBSP_UART_Debug_Printf("DSHOT300初始化失败\n");
        Error_Handler(); // 如果初始化失败,进入错误处理
    }

    // sBSP_TIM_DSHOT300_SetSpeedPercentage(10.0f, 10.0f, 10.0f, 10.0f); // 设置电调转速百分比为10%
    // sBSP_TIM_DSHOT300_SetEn(true);                                    // 启用DSHOT300

    // motor.init([](uint16_t lu1, uint16_t ru2, uint16_t ld3, uint16_t rd4) {
    //     sBSP_TIM_DSHOT300_SendPacket(lu1, ru2, ld3, rd4);
    // }); // 初始化电调驱动

    motor.init(sBSP_TIM_DSHOT300_SendPacket); // 设置发送数据包的函数指针

    log_printfln("Hello I'm sightseer's Quad Drone Flight Controller v1"); // 打印Hello World!

    sBSP_I2C1_Init(400000); // 初始化I2C1

    // 扫描一下所有I2C设备
    log_printfln("Scanning I2C devices...");
    for(uint8_t i = 0; i < 127; i++) {
        if(sBSP_I2C1M_DevIsReady(i << 1)) {
            sBSP_UART_Debug_Printf("I2C device found: 0x%02X\n", i);
        }
    }

    // sDRV_SPL06_Init();
    spl0601_init();
    spl0601_rateset(0, 128, 128); // 64Hz,over sampling 32
    spl0601_start_continuous(3);

    sAPP_ParamSave_Init();

    sAPP_CLI_Init();



    // 配置RUN LED引脚为输出
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = RUN_LED_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RUN_LED_GPIO_Port, &GPIO_InitStruct);

    ahrs.init(AHRS::IMUType::ICM45686, AHRS::MAGType::LIS3MDLTR); // 初始化AHRS

    // 读取IMU静态零偏
    sAPP_Tasks_ReadIMUCaliVal();

    sBSP_UART_Debug_Printf("Current free heap size: %u bytes\n", (unsigned int)xPortGetFreeHeapSize());

    sAPP_Tasks_CreateAll();
    sBSP_UART_Debug_Printf("Current free heap size: %u bytes\n", (unsigned int)xPortGetFreeHeapSize());
    sBSP_UART_Debug_Printf("FreeRTOS启动任务调度\n");
    vTaskStartScheduler();

    while(1) {
        // motor.setSpeedPercentage(10.0f, 2.0f, 2.0f, 2.0f);
        // motor.setSpeedPercentage(10.0f, 0.0f, 0.0f, 0.0f);
        // motor.setSpeedPercentage(3.0f, 3.0f, 3.0f, 3.0f);
        // motor.setSpeedPercentage(8.0f, 3.0f, 3.0f, 3.0f);
        // motor.setSpeedPercentage(13.0f, 13.0f, 13.0f, 13.0f);
        // motor.setSpeedPercentage(100.0f, 100.0f, 100.0f, 100.0f);

        // //梯形加减速测试
        // for(float speed = 0.0f; speed <= 100.0f; speed += 1.0f) {
        //     motor.setSpeedPercentage(speed, speed, speed, speed);
        //     HAL_Delay(20);
        // }

        // for(float speed = 100.0f; speed >= 0.0f; speed -= 1.0f) {
        //     motor.setSpeedPercentage(speed, speed, speed, speed);
        //     HAL_Delay(20);
        // }

        // SCK toggle
        //  HAL_GPIO_TogglePin(IMU_SCK_GPIO_Port,IMU_SCK_Pin); //翻转SCK引脚
        //  HAL_GPIO_TogglePin(IMU_MISO_GPIO_Port,IMU_MOSI_Pin); //翻转MISO引脚
        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin); // 翻转RUN LED引脚

        HAL_Delay(10);
    }
}
