#include "sBSP_I2C.h"

// I2C1的轮询超时时间
#define I2C1_POT_MS (100)
// I2C是否使用中断处理,默认使用轮询处理,此宏不影响Seq操作
// #define I2C1_USE_IT
// 用于中断式处理的延时
#define DELAY_MS(x) HAL_Delay(x)
// #define DELAY_MS(x)    vTaskDelay(x)
// 设置I2C一个ms能发多少字节,用于中断式传输等待,400K时,1ms能发送30字节
#define DELAY_MS_1MS_BYTES (30)

uint8_t I2C1M_RxCpltFlag;
uint8_t I2C1M_TxCpltFlag;

I2C_HandleTypeDef hi2c1;

/**
 * @brief  I2C1的初始化
 *
 * @param  ClkSpdHz I2C时钟频率
 *
 * @return 如果正常返回0,其他为异常值
 */
int8_t sBSP_I2C1_Init(uint32_t ClkSpdHz) {
    hi2c1.Instance        = I2C1;
    hi2c1.Init.ClockSpeed = ClkSpdHz;
    hi2c1.Init.DutyCycle  = I2C_DUTYCYCLE_2;
    // 从模式地址,主模式不需要
    hi2c1.Init.OwnAddress1 = 0;
    // 地址长度,从模式的设置
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    // 双地址模式,从模式的设置
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    // 第二个地址
    hi2c1.Init.OwnAddress2 = 0;
    // 广播模式,如果地址为0x00,那么所有设备都会响应(若可用)
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    // 时钟延展,从模式功能,作用:发送:你太快啦,等我一下
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

#ifdef I2C1_CHECK_ERROR
    if(HAL_I2C_Init(&hi2c1) == HAL_ERROR) { return -1; }
#else
    HAL_I2C_Init(&hi2c1);
#endif

    return 0;
}

/**
 * @brief  I2C1主机发送一个字节
 *
 * @param  DevAddr 设备地址
 * @param  data    要发送的数据
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SendByte(uint16_t DevAddr, uint8_t data) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Transmit_IT(&hi2c1, DevAddr, &data, 1);
    DELAY_MS(1);
    return ret;
#else
    return HAL_I2C_Master_Transmit(&hi2c1, DevAddr, &data, 1, I2C1_POT_MS);
#endif
}

/**
 * @brief  I2C1主机发送多个字节
 *
 * @param  DevAddr 设备地址
 * @param  pData   数据指针
 * @param  length  数据长度
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SendBytes(uint16_t DevAddr, uint8_t* pData, uint16_t length) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Transmit_IT(&hi2c1, DevAddr, pData, length);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
#else
    return HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, length, I2C1_POT_MS);
#endif
}

/**
 * @brief  I2C1主机接收一个字节
 *
 * @param  DevAddr 设备地址
 *
 * @return 返回的数据
 */
inline uint8_t sBSP_I2C1M_ReadByte(uint16_t DevAddr) {
    uint8_t ret = 0;
#ifdef I2C1_USE_IT
    // 这里必须舍弃函数的返回值来存数据了
    HAL_I2C_Master_Receive_IT(&hi2c1, DevAddr, &ret, 1);
    DELAY_MS(1);
#else
    HAL_I2C_Master_Receive(&hi2c1, DevAddr, &ret, 1, I2C1_POT_MS);
#endif
    return ret;
}

/**
 * @brief  I2C1主机接收多个字节
 *
 * @param  DevAddr 设备地址
 * @param  pData   数据指针
 * @param  length  数据长度
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_ReadBytes(uint16_t DevAddr, uint8_t* pData, uint16_t length) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Receive_IT(&hi2c1, DevAddr, pData, length);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
#else
    return HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, length, I2C1_POT_MS);
#endif
}

/**
 * @brief  I2C1主机顺序操作接收一个字节
 *
 * @param  DevAddr 设备地址
 * @param  XferOptions 操作,参考HAL库I2C @ref I2C_XferOptions_definition
 * @param  pData 接收到的数据
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SeqReadByte(uint16_t DevAddr, uint8_t* pData, uint32_t XferOptions) {
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Seq_Receive_IT(&hi2c1, DevAddr, pData, 1, XferOptions);
    DELAY_MS(1);
    return ret;
}

/**
 * @brief  I2C1主机顺序操作接收多个字节
 *
 * @param  DevAddr 设备地址
 * @param  pData   数据指针
 * @param  length  数据长度
 * @param  XferOptions 操作,参考HAL库I2C @ref I2C_XferOptions_definition
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SeqReadBytes(uint16_t DevAddr, uint8_t* pData, uint16_t length, uint32_t XferOptions) {
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Seq_Receive_IT(&hi2c1, DevAddr, pData, length, XferOptions);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
}

/**
 * @brief  I2C1主机顺序操作发送一个字节
 *
 * @param  DevAddr 设备地址
 * @param  XferOptions 操作,参考HAL库I2C @ref I2C_XferOptions_definition
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SeqSendByte(uint16_t DevAddr, uint8_t data, uint32_t XferOptions) {
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, DevAddr, &data, 1, XferOptions);
    DELAY_MS(1);
    return ret;
}

/**
 * @brief  I2C1主机顺序操作发送多个字节(中断式)
 *
 * @param  DevAddr 设备地址
 * @param  pData   数据指针
 * @param  length  数据长度
 * @param  XferOptions 操作,参考HAL库I2C @ref I2C_XferOptions_definition
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_SeqSendBytes(uint16_t DevAddr, uint8_t* pData, uint16_t length, uint32_t XferOptions) {
    uint8_t ret = 0;
    ret         = HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, DevAddr, pData, length, XferOptions);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
}

/**
 * @brief  I2C1主机对从机内存操作发送一个字节
 *
 * @param  dev_addr  设备地址
 * @param  mem_addr  内存地址
 * @param  addr_size 设备内存地址的长度,参数:I2C_MEMADD_SIZE_8BIT/I2C_MEMADD_SIZE_16BIT
 * @param  data      数据
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_MemSendByte(uint16_t dev_addr, uint16_t mem_addr, uint16_t addr_size, uint8_t data) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Mem_Write_IT(&hi2c1, dev_addr, mem_addr, addr_size, &data, 1);
    DELAY_MS(1);
    return ret;
#else
    return HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, addr_size, &data, 1, I2C1_POT_MS);
#endif
}

/**
 * @brief  I2C1主机对从机内存操作发送多个字节
 *
 * @param  dev_addr  设备地址
 * @param  mem_addr  内存地址
 * @param  addr_size 设备内存地址的长度,参数:I2C_MEMADD_SIZE_8BIT/I2C_MEMADD_SIZE_16BIT
 * @param  pData     数据指针
 * @param  length    长度
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_MemSendBytes(uint16_t dev_addr, uint16_t mem_addr, uint16_t addr_size, uint8_t* pData, uint16_t length) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Mem_Write_IT(&hi2c1, dev_addr, mem_addr, addr_size, pData, length);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
#else
    return HAL_I2C_Mem_Write(&hi2c1, dev_addr, mem_addr, addr_size, pData, length, I2C1_POT_MS);
#endif
}

/**
 * @brief  I2C1主机对从机内存操作接收一个字节
 *
 * @param  dev_addr  设备地址
 * @param  mem_addr  内存地址
 * @param  addr_size 设备内存地址的长度,参数:I2C_MEMADD_SIZE_8BIT/I2C_MEMADD_SIZE_16BIT
 *
 * @return 返回的数据
 */
inline uint8_t sBSP_I2C1M_MemReadByte(uint16_t dev_addr, uint16_t mem_addr, uint16_t addr_size) {
    uint8_t ret = 0;
#ifdef I2C1_USE_IT
    HAL_I2C_Mem_Read_IT(&hi2c1, dev_addr, mem_addr, addr_size, &ret, 1);
    DELAY_MS(1);
    return ret;
#else
    HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, addr_size, &ret, 1, I2C1_POT_MS);
    return ret;
#endif
}

/**
 * @brief  I2C1主机对从机内存操作接收多个字节
 *
 * @param  DevAddr 设备地址
 * @param  pData   数据指针
 * @param  length  数据长度
 *
 * @return 如果正常返回0,其他为异常值
 */
inline HAL_StatusTypeDef sBSP_I2C1M_MemReadBytes(uint16_t dev_addr, uint16_t mem_addr, uint16_t addr_size, uint8_t* pData, uint16_t length) {
#ifdef I2C1_USE_IT
    uint8_t ret = 0;
    ret         = HAL_I2C_Mem_Read_IT(&hi2c1, dev_addr, mem_addr, addr_size, pData, length);
    DELAY_MS(length / DELAY_MS_1MS_BYTES);
    return ret;
#else
    return HAL_I2C_Mem_Read(&hi2c1, dev_addr, mem_addr, addr_size, pData, length, I2C1_POT_MS);
#endif
}

inline int8_t sBSP_I2C1M_IsTxCplt() {
    return I2C1M_TxCpltFlag;
}

inline int8_t sBSP_I2C1M_IsRxCplt() {
    return I2C1M_RxCpltFlag;
}

inline int8_t sBSP_I2C1M_IsCplt() {
    return I2C1M_TxCpltFlag | I2C1M_RxCpltFlag;
}

bool sBSP_I2C1M_DevIsReady(uint16_t DevAddr) {
    // HAL库提供的函数,会自动发送一个读命令,如果没有ACK,则返回错误
    // 尝试5次
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c1, DevAddr, 5, I2C1_POT_MS);
    if(ret == HAL_OK) {
        return true;
    } else {
        return false;
    }
}

// void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
//     if(hi2c->Instance == I2C1) {
//         I2C1M_TxCpltFlag = 1;
//     }
// }

// void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
//     if(hi2c->Instance == I2C1) {
//         I2C1M_RxCpltFlag = 1;
//     }
// }

// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
//     if(hi2c->Instance == I2C1) {
//         // sHMI_Debug_Printf("sBSP_I2C:ERROR\n");
//     }
// }

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hi2c->Instance == I2C1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**I2C1 GPIO Configuration
        PB8     ------> I2C1_SCL
        PB9     ------> I2C1_SDA
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* I2C1 clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();

        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}
