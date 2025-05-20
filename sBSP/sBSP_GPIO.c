#include "sBSP_GPIO.h"

#include "sDRV_ICM45686.h"

#include "sBSP_UART.h"

#include "defines.h"





void sBSP_GPIO_IcmInt_Init(){
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_IT_FALLING;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_MEDIUM;
    gpio.Pin   = ICM_INT_Pin;
    HAL_GPIO_Init(ICM_INT_GPIO_Port,&gpio);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

#include "sUtils.h"

extern void sAPP_AHRS_ICMDataReadyCbISR();

void ISR_ATTR IRAM2_ATTR HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // log_printfln("GPIO_Pin: %d",GPIO_Pin);
    
    if(GPIO_Pin == ICM_INT_Pin){
        //通知AHRS,ICM的数据准备好了
        sAPP_AHRS_ICMDataReadyCbISR();
    }
}

// void ISR_ATTR IRAM2_ATTR HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    
//     if(GPIO_Pin == ICM_INT_Pin){
//         //通知AHRS,ICM的数据准备好了
//         extern void sAPP_AHRS_ICMDataReadyCbISR();
//         sAPP_AHRS_ICMDataReadyCbISR();
//     }
// }







