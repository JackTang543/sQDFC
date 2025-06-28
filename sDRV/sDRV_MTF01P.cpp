#include "sDRV_MTF01P.hpp"

sDRV_MTF01P optical_flow;

#include "sUtils.h"

sDRV_MTF01P::sDRV_MTF01P() {
    // 构造函数
}

sDRV_MTF01P::~sDRV_MTF01P() {
    // 析构函数
}

void sDRV_MTF01P::init() {
    // 初始化MTF-01P
    // 这里可以添加具体的初始化代码
}

int sDRV_MTF01P::getResult(result_t* res) {
    if(res == nullptr) return -1;           // 检查指针是否为nullptr
    memcpy(res, &result, sizeof(result_t)); // 复制结果到用户提供的结构体
    return 0;                               // 成功
}

void sDRV_MTF01P::recv_data_cb(uint8_t* data, uint16_t length) {
    /**
     * Micolink数据包格式:
     * 帧头: 0xEF
     * 设备ID: 0x0F
     * 系统ID: 0x00
     * 消息ID: 0x51
     * 包序列号: 0~0xFF
     * 负载长度:0x14
     * 数据负载:
     * uint32 系统时间 ms
     * uint32 激光测距距离 mm 最小值2,0表示无效
     * uint8  激光测距信号强度
     * uint8  距离精度
     * uint8  状态 1表示测距数据可用
     * uint8  reserve
     * int16  光流速度x轴:实际速度cm/s=光流速度*高度m
     * int16  光流速度y轴:实际速度cm/s=光流速度*高度m
     * uint8  光流质量,越大光流数据可信度越高
     * uint8  光流状态,0表示光流数据不可用,1表示光流数据可用
     * uint16 reserve
     * 帧校验: 前面所有数据的和
     */

    // 找到包头:0xEF
    uint8_t* packet_head = data;
    for(uint16_t i = 0; i < length - 1; i++) {
        // 包头固定0xEF 0x0F 0x00 0x51
        if(*packet_head == 0xEF && *(packet_head + 1) == 0x0F && *(packet_head + 2) == 0x00 && *(packet_head + 3) == 0x51) {
            // log_printfln("MTF-01P:找到包头:%u", i);
            break;
        }
        packet_head++;
    }
    // 没有找到包头
    if(packet_head >= data + length - 1) {
        log_error("MTF-01P:没有找到包头");
        return;
    }

    /*开始解析*/
    // 首先计算帧校验和
    uint8_t checksum              = 0;
    const uint16_t HEADER_LENGTH  = 6;    // 包头长度
    const uint16_t PAYLOAD_LENGTH = 0x14; // 负载长度20
    for(uint16_t i = 0; i < HEADER_LENGTH + PAYLOAD_LENGTH; i++) {
        checksum += *(packet_head + i);
    }
    // 检查校验和
    if(checksum != *(packet_head + HEADER_LENGTH + PAYLOAD_LENGTH)) {
        log_error("MTF-01P:包校验和错误,期望:%02X,实际:%02X", *(packet_head + HEADER_LENGTH + PAYLOAD_LENGTH), checksum);
        return;
    }

    const uint16_t PAYLOAD_BEGIN_INDEX = 6;
    uint8_t* payload                   = packet_head + PAYLOAD_BEGIN_INDEX;
    memcpy(payload_buf, payload, PAYLOAD_LENGTH); // 复制负载数据到缓冲区

    micolink_payload* p_payload = (micolink_payload*)payload_buf;
    // 解析数据
    result.laser_dis         = p_payload->laser_dis_mm / 1000.0f;             // 转换为 m
    result.x_velo            = p_payload->x_velo * result.laser_dis / 100.0f; // 转换为 m/s
    result.y_velo            = p_payload->y_velo * result.laser_dis / 100.0f;
    result.optical_quality   = p_payload->optical_quality / 255.0f;       // 转换为 0~1
    result.laser_strength    = p_payload->laser_signal_strength / 255.0f; // 转换为 0~1
    result.distance_accuracy = p_payload->distance_accuracy;              // 距离精度
    result.is_laser_valid    = p_payload->status == 1;                    // 激光测距数据是否有效
    result.is_optical_valid  = p_payload->optical_status == 1;            // 光流数据是否有效

    // log_printfln("激光距离:%.2f m,激光强度:%.2f,精度:%u,状态:%u,光流X轴:%.2f m/s,光流Y轴:%.2f m/s,质量:%.2f,状态:%u",
    //                 result.laser_dis,
    //                 result.laser_strength,
    //                 result.distance_accuracy,
    //                 result.is_laser_valid,
    //                 result.x_velo,
    //                 result.y_velo,
    //                 result.optical_quality,
    //                 result.is_optical_valid);

    // log_printfln("%.2f,%.2f,%u,%u,%.2f,%.2f,%.2f,%u",
    //                 result.laser_dis,
    //                 result.laser_strength,
    //                 result.distance_accuracy,
    //                 result.is_laser_valid,
    //                 result.x_velo,
    //                 result.y_velo,
    //                 result.optical_quality,
    //                 result.is_optical_valid);
}
