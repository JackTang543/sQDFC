#pragma once



class sAPP_MAVLink {
public:
    sAPP_MAVLink();
    ~sAPP_MAVLink();

    void init();
    void periodicProcess();

    // MAVLink消息发送函数
    void sendHeartbeat();
    void sendAttitude();
    void sendRawImu();
    void sendSysStatus();
    void sendBatteryStatus();
    void sendGlobalPositionInt();
    void sendLocalPositionNed();


private:







};


extern sAPP_MAVLink data_radio;


