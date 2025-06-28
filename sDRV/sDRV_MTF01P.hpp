#pragma once

#include <cstdbool>
#include <cstdint>
#include <string>
#include "defines.h"

class sDRV_MTF01P {
public:
    struct result_t {
        float x_velo;              // X轴速度 m/s
        float y_velo;              // Y轴速度 m/s
        float laser_dis;           // Z轴距离 m(这是光流传感器的激光测距的结果)
        float optical_quality;     // 光流置信度 0~1 如果0则数据不可信,0.01~1是数据可信度
        float laser_strength;      // 激光信号强度 0~1 如果0则数据不可信,0.01~1是数据可信度
        uint8_t distance_accuracy; // 距离精度 0~255
        bool is_laser_valid;       // 激光测距数据是否有效
        bool is_optical_valid;     // 光流数据是否有效
    };

    sDRV_MTF01P();
    ~sDRV_MTF01P();

    void init();

    int getResult(result_t* res);

    void recv_data_cb(uint8_t* data, uint16_t length);

private:
    typedef struct __packed {
        uint32_t sys_time_ms;          // 系统时间 ms
        uint32_t laser_dis_mm;         // 激光测距距离 mm 最小值2,0表示无效
        uint8_t laser_signal_strength; // 激光测距信号强度
        uint8_t distance_accuracy;     // 距离精度
        uint8_t status;                // 状态 1表示测距数据可用
        uint8_t reserve;               // 保留字节
        int16_t x_velo;                // 光流速度x轴:实际速度cm/s=光流速度*高度m
        int16_t y_velo;                // 光流速度y轴:实际速度cm/s=光流速度*高度m
        uint8_t optical_quality;       // 光流质量,越大光流数据可信度越高
        uint8_t optical_status;        // 光流状态,0表示光流数据不可用,1表示光流数据可用
        uint16_t reserve2;             // 保留字节
    } micolink_payload;

    result_t result; // 光流传感器的结果

    uint8_t payload_buf[32]; // 用于存储接收到的负载数据

    bool is_result_lock;
};

extern sDRV_MTF01P optical_flow; // 全局实例
