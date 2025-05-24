#include "sDRV_AHT20.h"


/**
 * sDRV_AHT20.c
 * BySightseer. inHNIP9607 250217
 */


#define AHT20_I2C_ADDR (0x70u)



static void portSendByte(uint8_t byte){
    sBSP_I2C1M_SendByte(AHT20_I2C_ADDR,byte);
}

static void portSendBytes(uint8_t* p_data,uint16_t len){
    sBSP_I2C1M_SendBytes(AHT20_I2C_ADDR,p_data,len);
}

static uint8_t portReadByte(){
    return sBSP_I2C1M_ReadByte(AHT20_I2C_ADDR);
}

static void portReadBytes(uint8_t* p_data,uint16_t len){
    sBSP_I2C1M_ReadBytes(AHT20_I2C_ADDR,p_data,len);
}


const static uint8_t measure_sequence[] = {0xAC,0x33,0x00};


void sDRV_AHT20_Init(){

}



void sDRV_AHT20_StartMeasure(){
    portSendBytes(measure_sequence,3);
}


void sDRV_AHT20_GetMeasure(float* temp,float* humi){
    uint8_t recv_sequence[7];
    portReadBytes(recv_sequence,7);

    float raw_humi = (recv_sequence[1] << 12) | (recv_sequence[2] << 4) | (recv_sequence[3] >> 4);
    float raw_temp = ((recv_sequence[3] & 0x0F) << 16) | (recv_sequence[4] << 8) | recv_sequence[5];

    *humi = (raw_humi * 100.0) / (1 << 20);
    *temp = ((raw_temp * 200.0) / (1 << 20)) - 50; 
}







