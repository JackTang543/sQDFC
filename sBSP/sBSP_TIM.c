#include "sBSP_TIM.h"

#include "sAPP_Debug.h"
#include "defines.h"

/**
 * sBSP_TIM.c
 * for sQDFC
 * 250520 bySightseer.
 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);

/*TIM2_CH1 -> IMU_HEATER*/
TIM_HandleTypeDef htim2;
#define TIM2_CLK_FREQ (84000000u)
/*init value,pwm duty accuracy=0.1%,max frequency=84KHz*/
const uint32_t TIM2_ARR_VAL = 1000 - 1;
const uint32_t TIM2_PSC_VAL = 0;

/*Given the timer clock freq,target freq,and ARR value,auto calculate the required PSC value*/
#define __TIM_GET_PSC(__TIM_CLK_FREQ, __TARGET_FREQ, __TIM_ARR_VAL) \
    ((__TIM_CLK_FREQ / (__TARGET_FREQ * (__TIM_ARR_VAL + 1))) - 1)

int sBSP_TIM_IMUHeater_Init() {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC          = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = TIM2_PSC_VAL;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = TIM2_ARR_VAL;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        return -1;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        return -2;
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        return -3;
    }

    HAL_TIM_MspPostInit(&htim2);

    return 0;
}

void sBSP_TIM_IMUHeater_SetPWMFreq(uint32_t freq) {
    // Given ARR=999,input freq range:2Hz to 84KHz
    __HAL_TIM_SET_PRESCALER(&htim2, __TIM_GET_PSC(TIM2_CLK_FREQ, freq, TIM2_ARR_VAL));
}

void sBSP_TIM_IMUHeater_SetEN(bool is_en) {
    is_en ? HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) : HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void sBSP_TIM_IMUHeater_SetDuty(float percent) {
    // 限制占空比0~10%之间,因为这个发热很大
    if(percent > 10.0f) { percent = 10.0f; }
    if(percent < 0.0f) { percent = 0.0f; }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(percent * 10.0f));
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(timHandle->Instance == TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM2 GPIO Configuration
        PA15     ------> TIM2_CH1
        */
        GPIO_InitStruct.Pin       = IMU_HEATER_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(IMU_HEATER_GPIO_Port, &GPIO_InitStruct);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle) {
    if(tim_pwmHandle->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}
