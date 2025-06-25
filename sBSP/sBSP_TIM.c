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
/*init value,pwm duty accuracy=0.1%,max frequency=142KHz*/
const static uint32_t TIM2_ARR_VAL = 1000 - 1;
const static uint32_t TIM2_PSC_VAL = 0;

/*TIM1 -> ESC DSHOT300*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;
DMA_HandleTypeDef hdma_tim1_ch3;
DMA_HandleTypeDef hdma_tim1_ch4;
#define TIM1_CLK_FREQ (168000000u)
// DSHOT300 300KBit/s PWM频率=300KHz 2.5%精度
const static uint32_t TIM1_PSC_VAL = 13;
// 1bit=3.33us HIGH=75%=2.5us=30 LOW=37.5%=1.25us=15
const static uint32_t TIM1_ARR_VAL = 39;
#define DSHOT300_BIT_HIGH (30) // DSHOT300 HIGH时长:2.5us
#define DSHOT300_BIT_LOW  (15) // DSHOT300 LOW时长:1.25us
// DSHOT300 时序:16bit+4delay(13us)
static uint16_t IRAM1_ATTR lu1_esc_timing[20];
static uint16_t IRAM1_ATTR ru2_esc_timing[20];
static uint16_t IRAM1_ATTR ld3_esc_timing[20];
static uint16_t IRAM1_ATTR rd4_esc_timing[20];

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

int sBSP_TIM_DSHOT300_Init() {
    TIM_ClockConfigTypeDef sClockSourceConfig           = {0};
    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = TIM1_PSC_VAL;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = TIM1_ARR_VAL;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if(HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        return -1;
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if(HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        return -1;
    }
    if(HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        return -1;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        return -1;
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    // sConfigOC.Pulse        = TIM1_ARR_VAL + 1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        return -1;
    }
    if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        return -1;
    }
    if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        return -1;
    }
    if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        return -1;
    }

    // sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    // sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    // sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    // sBreakDeadTimeConfig.DeadTime         = 0;
    // sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    // sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    // sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    // if(HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    //     return -1;
    // }

    HAL_TIM_MspPostInit(&htim1);

    return 0;
}

void sBSP_TIM_DSHOT300_SetEn(bool is_en) {
    if(is_en) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    }
}

void sBSP_TIM_DSHOT300_SendPacket(uint16_t lu1, uint16_t ru2, uint16_t ld3, uint16_t rd4) {
    // 转换为DSHOT时序
    for(uint32_t i = 0; i < 16; i++) {
        lu1_esc_timing[i] = (lu1 >> (15 - i) & 1) ? DSHOT300_BIT_HIGH : DSHOT300_BIT_LOW;
        ru2_esc_timing[i] = (ru2 >> (15 - i) & 1) ? DSHOT300_BIT_HIGH : DSHOT300_BIT_LOW;
        ld3_esc_timing[i] = (ld3 >> (15 - i) & 1) ? DSHOT300_BIT_HIGH : DSHOT300_BIT_LOW;
        rd4_esc_timing[i] = (rd4 >> (15 - i) & 1) ? DSHOT300_BIT_HIGH : DSHOT300_BIT_LOW;
    }

    // 添加4个延时位
    for(uint32_t i = 16; i < 20; i++) {
        lu1_esc_timing[i] = 0; // 延时位为低电平
        ru2_esc_timing[i] = 0;
        ld3_esc_timing[i] = 0;
        rd4_esc_timing[i] = 0;
    }

    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*)lu1_esc_timing, 20);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)ru2_esc_timing, 20);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t*)ld3_esc_timing, 20);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t*)rd4_esc_timing, 20);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(timHandle->Instance == TIM1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        PA9     ------> TIM1_CH2
        PA10     ------> TIM1_CH3
        PA11     ------> TIM1_CH4
        */
        GPIO_InitStruct.Pin       = ESC_LU1_Pin | ESC_RU2_Pin | ESC_LD3_Pin | ESC_RD4_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    } else if(timHandle->Instance == TIM2) {
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

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(tim_baseHandle->Instance == TIM1) {
        /* TIM1 clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        // __HAL_RCC_GPIOB_CLK_ENABLE();
        // /**TIM1 GPIO Configuration
        // PB12     ------> TIM1_BKIN
        // */
        // GPIO_InitStruct.Pin       = BKIN_Pin;
        // GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        // GPIO_InitStruct.Pull      = GPIO_NOPULL;
        // GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        // GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        // HAL_GPIO_Init(BKIN_GPIO_Port, &GPIO_InitStruct);

        /* TIM1 DMA Init */
        /* TIM1_CH1 Init */
        hdma_tim1_ch1.Instance                 = DMA2_Stream1;
        hdma_tim1_ch1.Init.Channel             = DMA_CHANNEL_6;
        hdma_tim1_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tim1_ch1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tim1_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_tim1_ch1.Init.Mode                = DMA_NORMAL;

        hdma_tim1_ch1.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_tim1_ch1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_CC1], hdma_tim1_ch1);

        /* TIM1_CH2 Init */
        hdma_tim1_ch2.Instance                 = DMA2_Stream2;
        hdma_tim1_ch2.Init.Channel             = DMA_CHANNEL_6;
        hdma_tim1_ch2.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_ch2.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tim1_ch2.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tim1_ch2.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_tim1_ch2.Init.Mode                = DMA_NORMAL;
        hdma_tim1_ch2.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_tim1_ch2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_tim1_ch2) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_CC2], hdma_tim1_ch2);

        /* TIM1_CH3 Init */
        hdma_tim1_ch3.Instance                 = DMA2_Stream6;
        hdma_tim1_ch3.Init.Channel             = DMA_CHANNEL_6;
        hdma_tim1_ch3.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_ch3.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tim1_ch3.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tim1_ch3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tim1_ch3.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_tim1_ch3.Init.Mode                = DMA_NORMAL;
        hdma_tim1_ch3.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_tim1_ch3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_tim1_ch3) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_CC3], hdma_tim1_ch3);

        /* TIM1_CH4_TRIG_COM Init */
        hdma_tim1_ch4.Instance                 = DMA2_Stream4;
        hdma_tim1_ch4.Init.Channel             = DMA_CHANNEL_6;
        hdma_tim1_ch4.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_tim1_ch4.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_tim1_ch4.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_tim1_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_tim1_ch4.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_tim1_ch4.Init.Mode                = DMA_NORMAL;
        hdma_tim1_ch4.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_tim1_ch4.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_tim1_ch4) != HAL_OK) {
            Error_Handler();
        }

        /* Several peripheral DMA handle pointers point to the same DMA handle.
         Be aware that there is only one stream to perform all the requested DMAs. */
        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_CC4], hdma_tim1_ch4);
        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_TRIGGER], hdma_tim1_ch4);
        __HAL_LINKDMA(tim_baseHandle, hdma[TIM_DMA_ID_COMMUTATION], hdma_tim1_ch4);

        HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(tim_pwmHandle->Instance == TIM1) {

    } else if(tim_pwmHandle->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void TIM1_BRK_TIM9_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

void TIM1_UP_TIM10_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

void DMA2_Stream1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch1);
}
void DMA2_Stream2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch2);
}
void DMA2_Stream6_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch3);
}
void DMA2_Stream4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_tim1_ch4);
}
