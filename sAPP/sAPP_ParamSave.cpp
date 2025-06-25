#include "sAPP_ParamSave.hpp"

#define MAGIC_NUM                 (0xACF7u)


#define ADDR_IMU_SBIAS            (0x0010u)
typedef struct __packed{
    float acc_x,acc_y,acc_z;
    float gyr_x,gyr_y,gyr_z;
    float temp;
    bool is_calibrated;
    uint16_t magic;
}imu_bias_t;
static imu_bias_t imu_bias;
#define SIZE_IMU_BIAS          (sizeof(imu_bias_t))

/*Gyroscope static zero bias*/
#define ADDR_GYR_SBIAS            (ADDR_IMU_SBIAS + SIZE_IMU_BIAS)
typedef struct __packed{
    float x_bias,y_bias,z_bias;
    float temp;
    uint16_t magic;
}gyr_bias_t;
static gyr_bias_t gyr_bias;
#define SIZE_GYR_BIAS          (sizeof(gyr_bias_t))

/*Accelerator static zero bias*/
#define ADDR_ACC_SBIAS            (ADDR_GYR_SBIAS + SIZE_GYR_BIAS)
typedef struct __packed{
    float x_bias,y_bias,z_bias;
    float temp;
    uint16_t magic;
}acc_bias_t;
static acc_bias_t acc_bias;
#define SIZE_ACC_BIAS          (sizeof(acc_bias_t))




static inline uint8_t portReadByte(uint16_t addr){
    return sDRV_MB85RCxx_ReadByte(addr);
}

static inline uint8_t portReadBytes(uint16_t addr,void* pData,uint16_t len){
    return sDRV_MB85RCxx_ReadBytes(addr,(uint8_t*)pData,len);
}

static inline uint8_t portWriteByte(uint16_t addr,uint8_t byte){
    return sDRV_MB85RCxx_WriteByte(addr,byte);
}

static inline uint8_t portWriteBytes(uint16_t addr,void* pData,uint16_t len){
    return sDRV_MB85RCxx_WriteBytes(addr,(uint8_t*)pData,len);
}




void sAPP_ParamSave_Init(){
    sDRV_MB85RCxx_Init();
    // sDRV_MB85RCxx_Format(0);
    // uint8_t buf[128];
    // memset(buf,0,128);
    // sDRV_MB85RCxx_WriteBytes(0,buf,128);
}



void sAPP_ParamSave_CaliIMU(){
    log_info("将在1s后进行IMU零偏校准...\n");
    vTaskDelay(1000);
    // ahrs.calcBias();
    imu_bias.acc_x = ahrs.imu_sbias.acc_x;
    imu_bias.acc_y = ahrs.imu_sbias.acc_y;
    imu_bias.acc_z = ahrs.imu_sbias.acc_z;
    imu_bias.gyr_x = ahrs.imu_sbias.gyr_x;
    imu_bias.gyr_y = ahrs.imu_sbias.gyr_y;
    imu_bias.gyr_z = ahrs.imu_sbias.gyr_z;
    imu_bias.temp  = ahrs.raw_data.imu_temp;
    imu_bias.is_calibrated = true;
    imu_bias.magic = MAGIC_NUM;
    log_info("校准完成! %.1f摄氏度下的校准值:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",\
    imu_bias.temp,imu_bias.acc_x,imu_bias.acc_y,imu_bias.acc_z,imu_bias.gyr_x,imu_bias.gyr_y,imu_bias.gyr_z);
    portWriteBytes(ADDR_IMU_SBIAS,&imu_bias,sizeof(imu_bias));
}


/// @brief 保存陀螺仪静态零偏数据到FeRAM
/// @param x_bias 单位:deg/s
/// @param y_bias 单位:deg/s
/// @param z_bias 单位:deg/s
/// @param temp   单位:摄氏度
/// @return 0:成功, -1:保存失败
/// @note 该函数会覆盖之前的静态零偏数据
int sAPP_ParamSave_SaveGyrSBias(float x_bias,float y_bias,float z_bias,float temp){
    gyr_bias.x_bias = x_bias;
    gyr_bias.y_bias = y_bias;
    gyr_bias.z_bias = z_bias;
    gyr_bias.temp  = temp;
    gyr_bias.magic = MAGIC_NUM;
    portWriteBytes(ADDR_GYR_SBIAS,&gyr_bias,sizeof(gyr_bias));
    return 0;
}

/// @brief 读取陀螺仪静态零偏数据
/// @param x_bias 单位:deg/s
/// @param y_bias 单位:deg/s
/// @param z_bias 单位:deg/s
/// @param temp   单位:摄氏度
/// @note 该函数会覆盖之前的静态零偏数据
/// @return 0:成功, -1:数据无效
int sAPP_ParamSave_ReadGyrSBias(float* x_bias,float* y_bias,float* z_bias,float* temp){
    portReadBytes(ADDR_GYR_SBIAS,&gyr_bias,sizeof(gyr_bias));
    if(gyr_bias.magic == MAGIC_NUM){
        *x_bias = gyr_bias.x_bias;
        *y_bias = gyr_bias.y_bias;
        *z_bias = gyr_bias.z_bias;
        *temp   = gyr_bias.temp;
        return 0;
    }else{
        return -1;
    }
}

/// @brief 保存加速度计静态零偏数据到FeRAM
/// @param x_bias 单位:m/s^2
/// @param y_bias 单位:m/s^2
/// @param z_bias 单位:m/s^2
/// @param temp   单位:摄氏度
/// @return 0:成功, -1:保存失败
int sAPP_ParamSave_SaveAccSBias(float x_bias,float y_bias,float z_bias,float temp){
    acc_bias.x_bias = x_bias;
    acc_bias.y_bias = y_bias;
    acc_bias.z_bias = z_bias;
    acc_bias.temp  = temp;
    acc_bias.magic = MAGIC_NUM;
    portWriteBytes(ADDR_ACC_SBIAS,&acc_bias,sizeof(acc_bias));
    return 0;
}

/// @brief 读取加速度计静态零偏数据
/// @param x_bias 单位:m/s^2
/// @param y_bias 单位:m/s^2
/// @param z_bias 单位:m/s^2
/// @param temp   单位:摄氏度
/// @return 0:成功, -1:数据无效
int sAPP_ParamSave_ReadAccSBias(float* x_bias,float* y_bias,float* z_bias,float* temp){
    portReadBytes(ADDR_ACC_SBIAS,&acc_bias,sizeof(acc_bias));
    if(acc_bias.magic == MAGIC_NUM){
        *x_bias = acc_bias.x_bias;
        *y_bias = acc_bias.y_bias;
        *z_bias = acc_bias.z_bias;
        *temp   = acc_bias.temp;
        return 0;
    }else{
        return -1;
    }
}







