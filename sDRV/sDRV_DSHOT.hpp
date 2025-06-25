#pragma once

#include "stdint.h"
#include "stdbool.h"

class sDRV_DSHOT {
public:
    // 指向bsp设置dshot packet函数的指针
    using SendPacketFunc = void (*)(uint16_t lu1, uint16_t ru2, uint16_t ld3, uint16_t rd4);

    sDRV_DSHOT();
    ~sDRV_DSHOT();

    // 初始化
    void init(SendPacketFunc send_packet);

    enum class CMD;

    // 设置电调转速百分比
    void setSpeedPercentage(float lu1, float ru2, float ld3, float rd4);

    void sendTelemetryCmd(CMD cmd) {
    }

    // dshot命令,设置命令时需要将遥测标志位置1
    enum class CMD {
        MOTOR_STOP = 0,               // Currently not implemented
        BEEP1,                        // Wait at least length of beep (380ms) before next command
        BEEP2,                        // Wait at least length of beep (380ms) before next command
        BEEP3,                        // Wait at least length of beep (400ms) before next command
        BEEP4,                        // Wait at least length of beep (400ms) before next command
        BEEP5,                        // Wait at least length of beep (400ms) before next command
        ESC_INFO,                     // Currently not implemented
        SPIN_DIRECTION_1,             // Need 6x, no wait required
        SPIN_DIRECTION_2,             // Need 6x, no wait required
        MODE_3D_OFF,                  // Need 6x, no wait required
        MODE_3D_ON,                   // Need 6x, no wait required
        SETTINGS_REQUEST,             // Currently not implemented
        SAVE_SETTINGS,                // Need 6x, wait at least 12ms before next command
        SPIN_DIRECTION_NORMAL   = 20, // Need 6x, no wait required
        SPIN_DIRECTION_REVERSED = 21, // Need 6x, no wait required
        LED0_ON,                      // Currently not implemented
        LED1_ON,                      // Currently not implemented
        LED2_ON,                      // Currently not implemented
        LED3_ON,                      // Currently not implemented
        LED0_OFF,                     // Currently not implemented
        LED1_OFF,                     // Currently not implemented
        LED2_OFF,                     // Currently not implemented
        LED3_OFF,                     // Currently not implemented
        MAX = 47
    };

private:


    uint16_t lu1_packet = 0;
    uint16_t ru2_packet = 0;
    uint16_t ld3_packet = 0;
    uint16_t rd4_packet = 0;

    uint16_t lu1_throttle = 0;
    uint16_t ru2_throttle = 0;
    uint16_t ld3_throttle = 0;
    uint16_t rd4_throttle = 0;

    SendPacketFunc send_packet_func = nullptr;

    static constexpr uint16_t throttle2dshot(uint16_t throttle) {
        uint16_t packet = throttle << 1;
        uint8_t crc     = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
        return (packet << 4) | crc;
    }
};

extern sDRV_DSHOT motor;

