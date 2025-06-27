#pragma once




#include "stm32f4xx_hal.h"



class sDWTLib{
public:

    sDWTLib();
    ~sDWTLib();

    int init(uint32_t _core_freq);

    void delay_us(uint32_t us);
    void delay_ms(uint32_t ms);

    void start();
    void end();
    uint32_t get_us();




private:

    uint32_t core_freq;
    //保存1us内的计数值
    uint32_t us_period;
    //保存1ms内的计数值
    uint32_t ms_period;
    //用于保存us级延时的数据
    uint32_t delay_us_val_start;
    uint32_t delay_us_val_end;
    //用于保存ms级延时的数据
    uint32_t delay_ms_val_start;
    uint32_t delay_ms_val_end;
    //用于保存测量值
    uint32_t measure_val_start;
    uint32_t measure_val_end;


};


extern sDWTLib dwt;






