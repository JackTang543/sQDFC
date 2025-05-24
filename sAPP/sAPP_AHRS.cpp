#include "sAPP_AHRS.hpp"

/**
 * sAPP_AHRS.cpp
 * 用于姿态估计
 *
 *
 * v1.1 241223 bySightseer.
 * ! 注意,因为改变了IMU数据获取方式,从直接获取变成了中断式获取,以降低latency
 * ! 所以,现在暂时不能用除ICM45686以外的其他IMU!!!
 *
 *
 */

#include "sBSP_SPI.h"
#include "ekf_AltEst6\ekf_AltEst6.h"


#include "sBSP_TIM.h"

#include "sBSP_UART.h"
#include "sAPP_Debug.h"

AHRS ahrs;

AHRS::AHRS() {
    memset(&imu_sbias, 0, sizeof(IMU_StaticBias));
}

int AHRS::init(IMUType imu_type, MAGType mag_type) {
    // 创建ICM数据就绪的二值信号量
    imu_data_ready = xSemaphoreCreateBinary();

    output.lock = xSemaphoreCreateMutex();

    // 创建EKF算法的状态数据的互斥锁
    ekf_altest6_info.lock = xSemaphoreCreateMutex();

    if(imu_data_ready == NULL || output.lock == NULL || ekf_altest6_info.lock == NULL) {
        log_error("AHRS:创建互斥锁失败");
        fatal_flag = FatalFlag::MUTEX_INIT_FATAL;
    }

    // 拉高ICM LIS3的CS引脚
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin              = ICM_CS_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull             = GPIO_PULLUP;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ICM_CS_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = LIS3_CS_Pin;
    HAL_GPIO_Init(LIS3_CS_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LIS3_CS_GPIO_Port, LIS3_CS_Pin, GPIO_PIN_SET);

    /*初始化通信接口*/
    if(imu_type == IMUType::ICM45686 || mag_type == MAGType::LIS3MDLTR) {
        // 10.5MBit/s
        if(sBSP_SPI_IMU_Init(SPI_BAUDRATEPRESCALER_8) != 0) {
            fatal_flag = FatalFlag::SPI_COMM_FATAL;
        }
    }

    /*初始化必要的6DoF器件*/
    if(imu_type == IMUType::ICM45686) {
        if(sDRV_ICM45686_Init() != 0) {
            fatal_flag = FatalFlag::IMU_INIT_FATAL;
        }
    }

    /*初始化磁力计*/
    if(mag_type == MAGType::LIS3MDLTR) {
        if(sDRV_LIS3_Init() != 0) {
            mag_state = MAGState::NO_MAG;
        }
    }

    ekf_AltEst6_init();

    // 初始化加热器PID
    sut_inc_pid_init(&heater_pid, 0.0f, 10.0f, -50.0f, 50.0f);
    //目标温度40摄氏度
    sut_inc_pid_setTar(&heater_pid, 40.0f);
    // 设置PID参数
    sut_inc_pid_SetK(&heater_pid, 1.0f, 0.5f, 0.0f);

    /*检查初始化标志位*/
    if(fatal_flag != FatalFlag::NONE || imu_state != IMUState::OK) {
        error_handler();
        return -1;
    }



    return 0;
}

void AHRS::error_handler() {
    log_error("AHRS:发生了错误");

    /*打印IMU类型和MAG类型*/
    switch(imu_type) {
        case IMUType::ICM45686:
            log_info("AHRS:IMU类型 ICM45686");
            break;
        default:
            Error_Handler();
            break;
    }

    switch(mag_type) {
        case MAGType::LIS3MDLTR:
            log_info("AHRS:磁力计类型 LIS3MDLTR");
            break;
        case MAGType::NONE:
            log_info("AHRS:无磁力计 NONE");
            break;
        default:
            Error_Handler();
            break;
    }

    /*检查致命错误标志位*/
    switch(fatal_flag) {
        case FatalFlag::NONE:
            log_info("AHRS:无致命错误 NONE");
            break;
        case FatalFlag::UNKNOW_FATAL:
            log_error("AHRS:未知致命错误 UNKNOW_FATAL");
            break;
        case FatalFlag::IMU_INIT_FATAL:
            log_error("AHRS:IMU初始化错误 IMU_INIT_FATAL");
            break;
        case FatalFlag::DT_MS_TOO_LARGE:
            log_error("AHRS:两次获取IMU的数据间隔时间过大,可能是IMU出错 DT_MS_TOO_LARGE");
            break;
        case FatalFlag::AE_ALGO_FATAL:
            log_error("AHRS:姿态估计算法内部错误 AE_ALGO_FATAL");
            break;
        case FatalFlag::IMU_FATAL:
            log_error("AHRS:IMU错误 IMU_FATAL");
            break;
        case FatalFlag::SPI_COMM_FATAL:
            log_error("AHRS:SPI通信错误 SPI_COMM_FATAL");
            break;
        case FatalFlag::MUTEX_FATAL:
            log_error("AHRS:互斥锁错误 MUTEX_FATAL");
            break;
        case FatalFlag::MUTEX_INIT_FATAL:
            log_error("AHRS:互斥锁初始化错误 MUTEX_INIT_FATAL");
            break;
        default:
            Error_Handler();
            break;
    }

    /*检查IMU状态*/
    switch(imu_state) {
        case IMUState::OK:
            log_info("AHRS:IMU状态正常 OK");
            break;
        case IMUState::NEED_CALIB:
            log_warn("AHRS:IMU需要校准 NEED_CALIB");
            break;
        default:
            Error_Handler();
            break;
    }

    /*检查磁力计状态*/
    switch(mag_state) {
        case MAGState::OK:
            log_info("AHRS:磁力计状态正常 OK");
            break;
        case MAGState::NEED_CALIB:
            log_warn("AHRS:磁力计需要校准 NEED_CALIB");
            break;
        case MAGState::NO_MAG:
            log_warn("AHRS:无磁力计 NO_MAG");
            break;
        case MAGState::DATA_DISTURBED:
            log_warn("AHRS:磁力计数据被干扰,不可信 DATA_DISTURBED");
            break;
        default:
            Error_Handler();
            break;
    }
}

int AHRS::calcBias(uint16_t points, IMU_StaticBias& imu_sbias) {
    float acc_x_accu  = 0;
    float acc_y_accu  = 0;
    float acc_z_accu  = 0;
    float gyro_x_accu = 0;
    float gyro_y_accu = 0;
    float gyro_z_accu = 0;
    for(uint16_t i = 0; i < points; i++) {
        // 减掉偏置是为了读到原始数据
        acc_x_accu += raw_data.acc_x;
        acc_y_accu += raw_data.acc_y;
        acc_z_accu += raw_data.acc_z;
        gyro_x_accu += raw_data.gyr_x;
        gyro_y_accu += raw_data.gyr_y;
        gyro_z_accu += raw_data.gyr_z;
        vTaskDelay(5);
    }
    imu_sbias.acc_x = acc_x_accu / points;
    imu_sbias.acc_y = acc_y_accu / points;
    imu_sbias.acc_z = acc_z_accu / points - M_GRAVITY; // 重力加速度 NED坐标系
    imu_sbias.gyr_x = gyro_x_accu / points;
    imu_sbias.gyr_y = gyro_y_accu / points;
    imu_sbias.gyr_z = gyro_z_accu / points;

    return 0;
}

void AHRS::updateAccSBias(float x_bias, float y_bias, float z_bias) {
    imu_sbias.acc_x = x_bias;
    imu_sbias.acc_y = y_bias;
    imu_sbias.acc_z = z_bias;
}

void AHRS::updateGyrSBias(float x_bias, float y_bias, float z_bias) {
    imu_sbias.gyr_x = x_bias;
    imu_sbias.gyr_y = y_bias;
    imu_sbias.gyr_z = z_bias;
}

void AHRS::getIMUData() {
    static int count;

    if(imu_type == IMUType::ICM45686) {
        sDRV_ICM45686_GetData();
        raw_data.acc_x    = g_icm45686.acc_x;
        raw_data.acc_y    = g_icm45686.acc_y;
        raw_data.acc_z    = g_icm45686.acc_z;
        raw_data.gyr_x    = g_icm45686.gyr_x;
        raw_data.gyr_y    = g_icm45686.gyr_y;
        raw_data.gyr_z    = g_icm45686.gyr_z;
        raw_data.imu_temp = g_icm45686.temp;
    }

    if(mag_type == MAGType::LIS3MDLTR) {
        // 每0.05s获取一次磁力计数据
        if(count >= 10) {
            sDRV_LIS3_GetData();
            raw_data.mag_x    = g_lis3.mag_x;
            raw_data.mag_y    = g_lis3.mag_y;
            raw_data.mag_z    = g_lis3.mag_z;
            raw_data.mag_temp = g_lis3.temp;
            count             = 0;
        } else {
            count++;
        }
    }
}


// AHRS任务,非阻塞式获取数据
void sAPP_AHRS_Task(void* param) {
    TickType_t xLastWakeTime;
    xLastWakeTime  = xTaskGetTickCount();
    float state[5] = {0};

    bool is_first = true;

    static uint32_t last_ts_ms = 0;
    static uint32_t ts_ms      = 0;
    static uint32_t dt_ms      = 0;

    for(;;) {
        // 当数据准备就绪则运行AHRS算法,此步骤消耗时间60us
        if(xSemaphoreTake(ahrs.imu_data_ready, 200) == pdTRUE) {

            /*获取原始数据*/
            ahrs.getIMUData();

            /*复制温度数据*/
            ahrs.output.imu_temp = ahrs.raw_data.imu_temp;
            ahrs.output.mag_temp = ahrs.raw_data.mag_temp;

            /*对IMU进行零偏校准*/
            ahrs.output.acc_x = ahrs.raw_data.acc_x - ahrs.imu_sbias.acc_x;
            ahrs.output.acc_y = ahrs.raw_data.acc_y - ahrs.imu_sbias.acc_y;
            ahrs.output.acc_z = ahrs.raw_data.acc_z - ahrs.imu_sbias.acc_z;
            ahrs.output.gyr_x = ahrs.raw_data.gyr_x - ahrs.imu_sbias.gyr_x;
            ahrs.output.gyr_y = ahrs.raw_data.gyr_y - ahrs.imu_sbias.gyr_y;
            ahrs.output.gyr_z = ahrs.raw_data.gyr_z - ahrs.imu_sbias.gyr_z;

            /*对磁力计进行校准*/
            // 硬磁校准
            ahrs.output.mag_x = ahrs.output.mag_x - ahrs.mag_cali.hard[0];
            ahrs.output.mag_y = ahrs.output.mag_y - ahrs.mag_cali.hard[1];
            ahrs.output.mag_z = ahrs.output.mag_z - ahrs.mag_cali.hard[2];
            // 软磁校准
            ahrs.output.mag_x = ahrs.mag_cali.soft[0] * ahrs.raw_data.mag_x + ahrs.mag_cali.soft[1] * ahrs.raw_data.mag_y + ahrs.mag_cali.soft[2] * ahrs.raw_data.mag_z;
            ahrs.output.mag_y = ahrs.mag_cali.soft[3] * ahrs.raw_data.mag_x + ahrs.mag_cali.soft[4] * ahrs.raw_data.mag_y + ahrs.mag_cali.soft[5] * ahrs.raw_data.mag_z;
            ahrs.output.mag_z = ahrs.mag_cali.soft[6] * ahrs.raw_data.mag_x + ahrs.mag_cali.soft[7] * ahrs.raw_data.mag_y + ahrs.mag_cali.soft[8] * ahrs.raw_data.mag_z;

            /*给姿态解算准备数据*/
            float input_gyr[3] = {ahrs.output.gyr_x, ahrs.output.gyr_y, ahrs.output.gyr_z};
            float input_acc[3] = {ahrs.output.acc_x, ahrs.output.acc_y, ahrs.output.acc_z};
            float input_mag[3] = {ahrs.output.mag_x, ahrs.output.mag_y, ahrs.output.mag_z};
            float eul[3]       = {0};
            float quat[4]      = {0};

            /*计算两次调用的时间间隔*/
            if(!is_first) {
                last_ts_ms = ts_ms;
                ts_ms      = HAL_GetTick();
                dt_ms      = ts_ms - last_ts_ms;
            } else {
                last_ts_ms = HAL_GetTick();
                ts_ms      = last_ts_ms;
                is_first   = false;
            }

            // 超时则报错
            if(dt_ms > 50) {
                dt_ms           = 50;
                ahrs.fatal_flag = AHRS::FatalFlag::DT_MS_TOO_LARGE;
                ahrs.error_handler();
            }



            // //给互补滤波准备数据
            // ahrs.input.acc_x  = ahrs.dat.acc_x;
            // ahrs.input.acc_y  = ahrs.dat.acc_y;
            // ahrs.input.acc_z  = ahrs.dat.acc_z;
            // ahrs.input.gyro_x = ahrs.dat.gyr_x;
            // ahrs.input.gyro_y = ahrs.dat.gyr_y;
            // ahrs.input.gyro_z = ahrs.dat.gyr_z;
            // 融合算法
            // sLib_6AxisCompFilter(&ahrs.input, &ahrs.result);
            // complementary_filter(&ahrs.input, &ahrs.result);

            float dt = dt_ms / 1000.0f;
            // MadgwickAHRSupdate(input_gyr[0] * DEG2RAD, input_gyr[1] * DEG2RAD, input_gyr[2] * DEG2RAD, input_acc[0], input_acc[1], input_acc[2], input_mag[0], input_mag[1], input_mag[2]);
            // MadgwickAHRSupdateIMU(input_gyr[0] * DEG2RAD, input_gyr[1] * DEG2RAD, input_gyr[2] * DEG2RAD, input_acc[0], input_acc[1], input_acc[2]);
            //四元数转换欧拉角
            //把上面的全部替换成q0 q1这样
            // eul[0] = atan2f(2.0F * (q0 * q1 + q2 * q3), 1.0F - 2.0F * (q1 * q1 + q2 * q2)) * RAD2DEG;
            // eul[1] = asinf(2.0F * (q0 * q2 - q1 * q3)) * RAD2DEG;
            // eul[2] = atan2f(2.0F * (q0 * q3 + q1 * q2), 1.0F - 2.0F * (q2 * q2 + q3 * q3)) * RAD2DEG;
            // quat[0] = q0;
            // quat[1] = q1;
            // quat[2] = q2;
            // quat[3] = q3;
            // eul[0] = atan2f(2.0F * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0F - 2.0F * (quat[1] * quat[1] + quat[2] * quat[2]));
            
            // MadgwickAHRSupdate(input_gyr[0], input_gyr[1], input_gyr[2], input_acc[0], input_acc[1], input_acc[2],input_mag[0], input_mag[1], input_mag[2]);
            ekf_AltEst6(input_gyr, input_acc, 2, dt, eul, quat, state);
            // sBSP_UART_Debug_Printf("%u\n",dwt.get_us());
            // sBSP_UART_Debug_Printf("%.3f,%.3f,%.3f,%u\n",ahrs.raw_data.gyr_x,ahrs.raw_data.gyr_y,ahrs.raw_data.gyr_z,HAL_GetTick());
            // sBSP_UART_Debug_Printf("%.3f,%.3f,%.3f,%u\n",ahrs.raw_data.acc_x,ahrs.raw_data.acc_y,ahrs.raw_data.acc_z,HAL_GetTick());
            // sBSP_UART_Debug_Printf("%u,%u\n",HAL_GetTick(),dwt.get_us());

            // ahrs.dat.pitch = ahrs.result.pitch;
            // ahrs.dat.roll  = ahrs.result.roll;
            // ahrs.dat.yaw   = ahrs.result.yaw;
            // ahrs.dat.q0    = ahrs.result.q0;
            // ahrs.dat.q1    = ahrs.result.q1;
            // ahrs.dat.q2    = ahrs.result.q2;
            // ahrs.dat.q3    = ahrs.result.q3;

            // sBSP_UART_Debug_Printf("%.2f,%.2f,%.2f,%.6f,%.6f,",\
            eul[0],eul[1],eul[2],bias[0],bias[1]);
            // sBSP_UART_Debug_Printf("%u,%u\n",HAL_GetTick(),dwt.get_us());

            ahrs.output.pitch = eul[0];
            ahrs.output.roll  = eul[1];
            ahrs.output.yaw   = eul[2];
            ahrs.output.q0    = quat[0];
            ahrs.output.q1    = quat[1];
            ahrs.output.q2    = quat[2];
            ahrs.output.q3    = quat[3];

            // 打印出欧拉角
            // sBSP_UART_Debug_Printf("%.2f,%.2f,%.2f\n", ahrs.output.pitch, ahrs.output.roll, ahrs.output.yaw);

            // sBSP_UART_Debug_Printf("%.2f\n",ahrs.raw_data.imu_temp);
            // sBSP_UART_Debug_Printf("%u,%u\n",HAL_GetTick(),dwt.get_us());


            // float yaw = atan2f(ahrs.dat.mag_y,ahrs.dat.mag_x) * RAD2DEG;
            // float yaw = atan2f(ahrs.dat.mag_x,ahrs.dat.mag_y) * RAD2DEG;

            // sBSP_UART_Debug_Printf("%.2f,%.2f,%.2f,%.2f,%u\n",ahrs.dat.mag_x,ahrs.dat.mag_y,ahrs.dat.mag_z,yaw,HAL_GetTick());
            // 把新获取到的数据通过队列发送给blc_ctrl算法
            // xQueueSend(g_blc_ctrl_ahrs_queue,&ahrs.dat,200);
            // xQueueOverwrite(g_blc_ctrl_ahrs_queue,&ahrs.output);

        }
        // 如果等待200ms还没有获取到信号量则报错
        else {
            log_error("AHRS错误:获取icm_data_ready_bin超时");
            Error_Handler();
        }

        ahrs.ekf_altest6_info.trace_R       = state[0];
        ahrs.ekf_altest6_info.trace_P       = state[1];
        ahrs.ekf_altest6_info.chi_square    = state[2];
        ahrs.ekf_altest6_info.trace_acc_err = state[3];
        ahrs.ekf_altest6_info.acc_norm      = state[4];

        // sBSP_UART_Debug_Printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,",ahrs.dat.acc_x,ahrs.dat.acc_y,ahrs.dat.acc_z,ahrs.dat.gyr_x,ahrs.dat.gyr_y,ahrs.dat.gyr_z,ahrs.dat.mag_x,ahrs.dat.mag_y,ahrs.dat.mag_z);
        // sBSP_UART_Debug_Printf("%u\n",HAL_GetTick());
        // 高精确度延时10ms
        // xTaskDelayUntil(&xLastWakeTime,10 / portTICK_PERIOD_MS);
    }
}


//周期20ms 50Hz
void AHRS::periodicProcess(){
    /*计算恒温控制PID*/
    heater_pwm_duty = sut_inc_pid_update(&heater_pid, ahrs.raw_data.imu_temp, 0.02f);
    sBSP_TIM_IMUHeater_SetDuty(heater_pwm_duty);
    // sBSP_UART_Debug_Printf("%.2f,%.2f\n",ahrs.raw_data.imu_temp, heater_pwm_duty);
}



void IRAM2_ATTR sAPP_AHRS_ICMDataReadyCbISR() {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    // 通知数据就绪
    xSemaphoreGiveFromISR(ahrs.imu_data_ready, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken) portYIELD();
}



