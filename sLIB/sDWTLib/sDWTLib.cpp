#include "sDWTLib.hpp"


/**
 * sDWTLib
 * 
 * 241202 v1
 * 
 * 
 */



sDWTLib dwt;



sDWTLib::sDWTLib(){

}

sDWTLib::~sDWTLib(){

}


int sDWTLib::init(uint32_t _core_freq){
    //使能DWT外设,置DEMCR寄存器的TRCENA位为1
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    //清空CYCCNT
    DWT->CYCCNT = 0;
    //写DWTCR寄存器的CYCCNTTENA位为1,启动CYCCNT计数
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    core_freq = _core_freq;
    
    //在系统频率下,1us和1ms里的DWT增加的值
    us_period = core_freq * 0.000001;
    ms_period = core_freq * 0.001;

    return 0;
}


void sDWTLib::delay_us(uint32_t us){
    delay_us_val_start = DWT->CYCCNT;
    delay_us_val_end = delay_us_val_start + (us * us_period);
    while(DWT->CYCCNT < delay_us_val_end){__NOP();}
}

void sDWTLib::delay_ms(uint32_t ms){
    delay_ms_val_start = DWT->CYCCNT;
    delay_ms_val_end = delay_ms_val_start + (ms * ms_period);
    while(DWT->CYCCNT < delay_ms_val_end){__NOP();}
}

void sDWTLib::start(){
    measure_val_start = DWT->CYCCNT;
}

void sDWTLib::end(){
    measure_val_end = DWT->CYCCNT;
}

uint32_t sDWTLib::get_us(){
    //处理发生边界溢出的情况
    if(measure_val_end <= measure_val_start){
        return ((0xFFFFFFFF - measure_val_start) + measure_val_end) / us_period;
    }
    //正常情况
    else{
        return (measure_val_end - measure_val_start) / us_period;
    }
}

