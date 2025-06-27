#include "sDRV_DSHOT.hpp"

#include "sUtils.h"


sDRV_DSHOT motor;

sDRV_DSHOT::sDRV_DSHOT() {
}

sDRV_DSHOT::~sDRV_DSHOT() {
    // 析构函数,如果有需要清理的资源可以在这里处理
}

void sDRV_DSHOT::init(SendPacketFunc send_packet) {

    // 设置发送数据包的函数指针
    send_packet_func = send_packet;
}

void sDRV_DSHOT::setSpeedPercentage(float lu1, float ru2, float ld3, float rd4) {
    // 限制速度0~100%之间
    sut_fconstrain(&lu1, 0.0f, 100.0f);
    sut_fconstrain(&ru2, 0.0f, 100.0f);
    sut_fconstrain(&ld3, 0.0f, 100.0f);
    sut_fconstrain(&rd4, 0.0f, 100.0f);

    // 将百分比转换为DSHOT格式的油门(48~2047也就是0~1999)
    lu1_throttle = static_cast<uint16_t>(sut_fmap(lu1, 0.0f, 100.0f, 48.0f, 2047.0f));
    ru2_throttle = static_cast<uint16_t>(sut_fmap(ru2, 0.0f, 100.0f, 48.0f, 2047.0f));
    ld3_throttle = static_cast<uint16_t>(sut_fmap(ld3, 0.0f, 100.0f, 48.0f, 2047.0f));
    rd4_throttle = static_cast<uint16_t>(sut_fmap(rd4, 0.0f, 100.0f, 48.0f, 2047.0f));


    // 转换为DSHOT格式的油门包
    lu1_packet = throttle2dshot(lu1_throttle);
    ru2_packet = throttle2dshot(ru2_throttle);
    ld3_packet = throttle2dshot(ld3_throttle);
    rd4_packet = throttle2dshot(rd4_throttle);

    // 发送数据包
    if(send_packet_func) {
        send_packet_func(lu1_packet, ru2_packet, ld3_packet, rd4_packet);
    }
}
