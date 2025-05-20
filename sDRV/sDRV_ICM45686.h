#pragma once
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"


//读出的数据
typedef struct{
    float    temp;         //温度,单位摄氏度
    float    acc_x;        //加速度数据,单位m/s^2
    float    acc_y;
    float    acc_z;
    float    gyr_x;       //陀螺仪数据,单位dps
    float    gyr_y;
    float    gyr_z;
    uint32_t tick;         //
}sDRV_ICM45686_Data_t;


//陀螺仪量程
typedef enum{
    SDRV_ICM45686_GYRO_UI_FS_SEL_4000DPS   = 0b0000,
    SDRV_ICM45686_GYRO_UI_FS_SEL_2000DPS   = 0b0001,
    SDRV_ICM45686_GYRO_UI_FS_SEL_1000DPS   = 0b0010,
    SDRV_ICM45686_GYRO_UI_FS_SEL_500DPS    = 0b0011,
    SDRV_ICM45686_GYRO_UI_FS_SEL_250DPS    = 0b0100,
    SDRV_ICM45686_GYRO_UI_FS_SEL_125DPS    = 0b0101,
    SDRV_ICM45686_GYRO_UI_FS_SEL_62D5DPS   = 0b0110,
    SDRV_ICM45686_GYRO_UI_FS_SEL_31D25DPS  = 0b0111,
    SDRV_ICM45686_GYRO_UI_FS_SEL_15D625DPS = 0b1000,
}sDRV_ICM45686_GYRO_UI_FS_SEL_t;

//陀螺仪ODR
typedef enum{
    SDRV_ICM45686_GYRO_ODR_6D4KHZ          = 0b0011,
    SDRV_ICM45686_GYRO_ODR_3D2KHZ          = 0b0100,
    SDRV_ICM45686_GYRO_ODR_1D6KHZ          = 0b0101,
    SDRV_ICM45686_GYRO_ODR_800HZ           = 0b0110,
    SDRV_ICM45686_GYRO_ODR_400HZ           = 0b0111,
    SDRV_ICM45686_GYRO_ODR_200HZ           = 0b1000,
    SDRV_ICM45686_GYRO_ODR_100HZ           = 0b1001,
    SDRV_ICM45686_GYRO_ODR_50HZ            = 0b1010,
    SDRV_ICM45686_GYRO_ODR_25HZ            = 0b1011,
    SDRV_ICM45686_GYRO_ODR_12D5HZ          = 0b1100,
    SDRV_ICM45686_GYRO_ODR_6D25HZ          = 0b1101,
    SDRV_ICM45686_GYRO_ODR_3D125HZ         = 0b1110,
    SDRV_ICM45686_GYRO_ODR_1D5625HZ        = 0b1111,
}sDRV_ICM45686_GYRO_ODR_t;

//加速度计满量程配置
typedef enum{
    SDRV_ICM45686_ACCEL_UI_FS_SEL_32G      = 0b000,
    SDRV_ICM45686_ACCEL_UI_FS_SEL_16G      = 0b001,
    SDRV_ICM45686_ACCEL_UI_FS_SEL_8G       = 0b010,
    SDRV_ICM45686_ACCEL_UI_FS_SEL_4G       = 0b011,
    SDRV_ICM45686_ACCEL_UI_FS_SEL_2G       = 0b100,
}sDRV_ICM45686_ACCEL_UI_FS_SEL_t;

//加速度计ODR
typedef enum{
    SDRV_ICM45686_ACCEL_ODR_6D4KHZ         = 0b0011,
    SDRV_ICM45686_ACCEL_ODR_3D2KHZ         = 0b0100,
    SDRV_ICM45686_ACCEL_ODR_1D6KHZ         = 0b0101,
    SDRV_ICM45686_ACCEL_ODR_800HZ          = 0b0110,
    SDRV_ICM45686_ACCEL_ODR_400HZ          = 0b0111,
    SDRV_ICM45686_ACCEL_ODR_200HZ          = 0b1000,
    SDRV_ICM45686_ACCEL_ODR_100HZ          = 0b1001,
    SDRV_ICM45686_ACCEL_ODR_50HZ           = 0b1010,
    SDRV_ICM45686_ACCEL_ODR_25HZ           = 0b1011,
    SDRV_ICM45686_ACCEL_ODR_12D5HZ         = 0b1100,
    SDRV_ICM45686_ACCEL_ODR_6D25HZ         = 0b1101,
    SDRV_ICM45686_ACCEL_ODR_3D125HZ        = 0b1110,
    SDRV_ICM45686_ACCEL_ODR_1D5625HZ       = 0b1111,
}sDRV_ICM45686_ACCEL_ODR_t;

typedef enum{
    SDRV_ICM45686_GYRO_MODE_OFF            = 0b00,
    SDRV_ICM45686_GYRO_MODE_STANDBY        = 0b01,
    SDRV_ICM45686_GYRO_MODE_LP             = 0b10,
    SDRV_ICM45686_GYRO_MODE_LN             = 0b11,
}sDRV_ICM45686_GYRO_MODE_t;

typedef enum{
    SDRV_ICM45686_ACCEL_MODE_OFF           = 0b00,
    SDRV_ICM45686_ACCEL_MODE_STANDBY       = 0b01,
    SDRV_ICM45686_ACCEL_MODE_LP            = 0b10,
    SDRV_ICM45686_ACCEL_MODE_LN            = 0b11,
}sDRV_ICM45686_ACCEL_MODE_t;

// //温度计DLPF设置
// typedef enum{
//     SDRV_ICM_TEMPFILTBW_4000HZ = 0b000,     //BW=4KHZ 延迟0.125ms(默认)
//     SDRV_ICM_TEMPFILTBW_170HZ  = 0b001,     //BW=170HZ 1ms
//     SDRV_ICM_TEMPFILTBW_82HZ   = 0b010,     //BW=82HZ 2ms
//     SDRV_ICM_TEMPFILTBW_40HZ   = 0b011,     //BW=40HZ 4ms
//     SDRV_ICM_TEMPFILTBW_20HZ   = 0b100,     //BW=20HZ 8ms
//     SDRV_ICM_TEMPFILTBW_10HZ   = 0b101,     //BW=10HZ 16ms
//     SDRV_ICM_TEMPFILTBW_5HZ    = 0b110,     //BW=5HZ  32ms
// }sDRV_ICM_TEMPFILTBW_t;

// //陀螺仪滤波阶数
// typedef enum{
//     SDRV_ICM_GYROUIFILTORD_1   = 0b00,      //1阶滤波器
//     SDRV_ICM_GYROUIFILTORD_2   = 0b01,      //2阶滤波器
//     SDRV_ICM_GYROUIFILTORD_3   = 0b10,      //3阶滤波器
// }sDRV_ICM_GYROUIFILTORD_t;

// //陀螺仪DEC2_M2滤波器阶数
// typedef enum{
//     SDRV_ICM_GYRODEC2M2ORD_3   = 0b10,      //3阶滤波器
// }sDRV_ICM_GYRODEC2M2ORD_t;

// //加速度计LPF滤波带宽
// typedef enum{
//     SDRV_ICM_ACCELUIFILTBW_LNODRF2       = 0,   //LN模式,BW=ODR/2
//     SDRV_ICM_ACCELUIFILTBW_LNODRF4       = 1,   //BW=max(400Hz, ODR)/4 (默认)
//     SDRV_ICM_ACCELUIFILTBW_LNODRF5       = 2,   //BW=max(400Hz, ODR)/5
//     SDRV_ICM_ACCELUIFILTBW_LNODRF8       = 3,   //BW=max(400Hz, ODR)/8
//     SDRV_ICM_ACCELUIFILTBW_LNODRF10      = 4,   //BW=max(400Hz, ODR)/10
//     SDRV_ICM_ACCELUIFILTBW_LNODRF16      = 5,   //BW=max(400Hz, ODR)/16
//     SDRV_ICM_ACCELUIFILTBW_LNODRF20      = 6,   //BW=max(400Hz, ODR)/20
//     SDRV_ICM_ACCELUIFILTBW_LNODRF40      = 7,   //BW=max(400Hz, ODR)/40
//     SDRV_ICM_ACCELUIFILTBW_LNLLODR       = 14,  //低延时模式Dec2 max(400Hz, ODR)
//     SDRV_ICM_ACCELUIFILTBW_LNLLODRM8     = 15,  //低延时模式Dec2 max(400Hz, 8*ODR)
//     SDRV_ICM_ACCELUIFILTBW_LP1AVG        = 1,   //LP模式 1个样本的平均滤波
//     SDRV_ICM_ACCELUIFILTBW_LP16AVG       = 6,   //LP模式 16个样本的平均滤波
// }sDRV_ICM_ACCELUIFILTBW_t;

// //陀螺仪LPF滤波带宽
// typedef enum{
//     SDRV_ICM_GYROUIFILTBW_LNODRF2       = 0,   //LN模式,BW=ODR/2
//     SDRV_ICM_GYROUIFILTBW_LNODRF4       = 1,   //BW=max(400Hz, ODR)/4 (默认)
//     SDRV_ICM_GYROUIFILTBW_LNODRF5       = 2,   //BW=max(400Hz, ODR)/5
//     SDRV_ICM_GYROUIFILTBW_LNODRF8       = 3,   //BW=max(400Hz, ODR)/8
//     SDRV_ICM_GYROUIFILTBW_LNODRF10      = 4,   //BW=max(400Hz, ODR)/10
//     SDRV_ICM_GYROUIFILTBW_LNODRF16      = 5,   //BW=max(400Hz, ODR)/16
//     SDRV_ICM_GYROUIFILTBW_LNODRF20      = 6,   //BW=max(400Hz, ODR)/20
//     SDRV_ICM_GYROUIFILTBW_LNODRF40      = 7,   //BW=max(400Hz, ODR)/40
//     SDRV_ICM_GYROUIFILTBW_LNLLODR       = 14,  //低延时模式Dec2 max(400Hz, ODR)
//     SDRV_ICM_GYROUIFILTBW_LNLLODRM8     = 15,  //低延时模式Dec2 max(400Hz, 8*ODR)
// }sDRV_ICM_GYROUIFILTBW_t;

// //加速度计滤波阶数
// typedef enum{
//     SDRV_ICM_ACCELUIFILTORD_1   = 0b00,      //1阶滤波器
//     SDRV_ICM_ACCELUIFILTORD_2   = 0b01,      //2阶滤波器
//     SDRV_ICM_ACCELUIFILTORD_3   = 0b10,      //3阶滤波器
// }sDRV_ICM_ACCELUIFILTORD_t;

// //加速度计DEC2_M2滤波器阶数
// typedef enum{
//     SDRV_ICM_ACCELDEC2M2ORD_3   = 0b10,      //3阶滤波器
// }sDRV_ICM_ACCELDEC2M2ORD_t;

// //选择寄存器bank
// typedef enum{
//     SDRV_ICM_REGBANKSEL_BANK0   = 0b000,     //默认
//     SDRV_ICM_REGBANKSEL_BANK1   = 0b001,
//     SDRV_ICM_REGBANKSEL_BANK2   = 0b010,
//     SDRV_ICM_REGBANKSEL_BANK3   = 0b011,
//     SDRV_ICM_REGBANKSEL_BANK4   = 0b100,
// }sDRV_ICM_REGBANKSEL_t;


//配置
typedef struct{
    sDRV_ICM45686_GYRO_UI_FS_SEL_t         gyro_fs;              //陀螺仪量程
    sDRV_ICM45686_GYRO_ODR_t               gyro_odr;             //陀螺仪输出速率
    sDRV_ICM45686_ACCEL_UI_FS_SEL_t        accel_fs;             //加速度计量程
    sDRV_ICM45686_ACCEL_ODR_t              accel_odr;            //加速度计输出速率
    sDRV_ICM45686_GYRO_MODE_t              gyro_mode;   
    sDRV_ICM45686_ACCEL_MODE_t             accel_mode;
}sDRV_ICM45686_Conf_t;


extern sDRV_ICM45686_Data_t g_icm45686;
extern sDRV_ICM45686_Conf_t g_icm45686_conf;




int sDRV_ICM45686_Init();
void sDRV_ICM45686_SetConfig(const sDRV_ICM45686_Conf_t* p_conf);


//如果使用中断式获取数据,在中断里调用这个函数
void sDRV_ICM45686_GetData();


#ifdef __cplusplus
}
#endif


