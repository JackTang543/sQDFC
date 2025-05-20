#pragma once

#include "math.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "sUtils.h"

#include "sDRV_ICM45686.h"
#include "sDRV_LIS3MDLTR.h"

#include "MadgwickAHRS.h"

void sAPP_AHRS_Task(void* param);

extern "C" void sAPP_AHRS_ICMDataReadyCbISR();

class AHRS {
public:
    AHRS();

    // IMU类型
    enum class IMUType {
        ICM45686 = 1,
    };

    // 磁力计类型
    enum class MAGType {
        NONE      = 0, // 无磁力计
        LIS3MDLTR = 1,
    };

    // 致命错误标志位
    enum class FatalFlag {
        NONE             = 0, // 无致命错误
        UNKNOW_FATAL     = 1, // 未知致命错误
        DT_MS_TOO_LARGE  = 2, // 两次获取IMU的数据间隔时间过大,可能是IMU出错
        AE_ALGO_FATAL    = 3, // 姿态估计算法内部错误
        IMU_FATAL        = 4, // IMU错误
        IMU_INIT_FATAL   = 5, // IMU初始化错误
        SPI_COMM_FATAL   = 6, // SPI通信错误
        MUTEX_FATAL      = 7, // 互斥锁错误
        MUTEX_INIT_FATAL = 8, // 互斥锁初始化错误
    };

    // 运行标志位
    enum class RunFlag {
        OK = 0, // 正常运行
    };

    // IMU状态
    enum class IMUState {
        OK         = 0, // 正常
        NEED_CALIB = 1, // 需要校准
    };

    // 磁力计状态
    enum class MAGState {
        OK             = 0, // 正常
        NEED_CALIB     = 1, // 需要校准
        NO_MAG         = 2, // 无磁力计,可能是磁力计初始化失败,或者没有连接磁力计
        DATA_DISTURBED = 3, // 可能数据被干扰,不可信,将不采纳磁力计的航向信息
    };

    // 来自IMU的数据
    struct RawData {
        float acc_x, acc_y, acc_z;
        float gyr_x, gyr_y, gyr_z;
        float mag_x, mag_y, mag_z;
        float imu_temp, mag_temp; // 温度,单位摄氏度
    };

    // IMU静态零偏数据
    struct IMU_StaticBias {
        float acc_x, acc_y, acc_z;
        float gyr_x, gyr_y, gyr_z;
    };

    // MAG校准数据
    struct MAG_CalibData {
        // 硬磁校准,偏置:x,y,z
        float hard[3];
        // 软磁校准,经过矩阵变换
        float soft[9];
    };

    // 姿态数据
    struct AHRSData {
        SemaphoreHandle_t lock;
        float acc_x, acc_y, acc_z;
        float gyr_x, gyr_y, gyr_z;
        float mag_x, mag_y, mag_z;
        float pitch, roll, yaw;
        float q0, q1, q2, q3;
        float imu_temp, mag_temp;
    };

    // EKF算法的状态数据
    struct EKF_ALTEST6_Info {
        SemaphoreHandle_t lock;
        float trace_R;
        float trace_P;
        float chi_square;
        float trace_acc_err;
        float acc_norm;
    };

    int init(IMUType imu_type, MAGType mag_type);

    int calcBias(uint16_t points, IMU_StaticBias& imu_sbias);
    void updateAccSBias(float x_bias, float y_bias, float z_bias);
    void updateGyrSBias(float x_bias, float y_bias, float z_bias);
    void getIMUData();

    // AHRS的周期调用的杂项事务
    void periodicProcess();

    // sLIB_6AXIS_INPUT_t input;
    // sLIB_ATTITUDE_RESULT_t result;

    // ICM数据就绪二值信号量
    SemaphoreHandle_t imu_data_ready;

    IMUType imu_type     = IMUType::ICM45686;
    MAGType mag_type     = MAGType::LIS3MDLTR;
    FatalFlag fatal_flag = FatalFlag::NONE;
    RunFlag run_flag     = RunFlag::OK;
    IMUState imu_state   = IMUState::OK;
    MAGState mag_state   = MAGState::OK;

    // IMU原始数据
    RawData raw_data;
    // IMU静态零偏数据
    IMU_StaticBias imu_sbias;
    // 磁力计校准数据
    MAG_CalibData mag_cali = {
        .hard = {5.226f, -9.756f, -115.873f},
        .soft = {
            1.0194f,
            0.0351f,
            0.0007f,
            0.0351f,
            0.9927f,
            0.0023f,
            0.0007f,
            0.0023f,
            0.9909f,
        },
    };
    // 姿态数据
    AHRSData output;
    // EKF算法的状态数据
    EKF_ALTEST6_Info ekf_altest6_info;

    void error_handler();

    IMUState getIMUState() {
        return imu_state;
    }

    void setIMUState(IMUState status) {
        imu_state = status;
    }

    IMUType getIMUType() {
        return imu_type;
    }
    MAGType getMAGType() {
        return mag_type;
    }

    sut_inc_pid_t heater_pid;
    float heater_pwm_duty = 0.0f; // 加热器的PWM占空比

private:
};

extern AHRS ahrs;
