#include "sDRV_ICM45686.h"


/**
 * sDRV_ICM45686.c
 * 
 * v1.0 241201
 * 
 */

#include "string.h"

#include "sUtils.h"


#include "sBSP_GPIO.h"

//重力常数
#define M_GRAVITY       (9.81398f)


/*************配置选项*********************/
//使用SPI接口
#define USE_SPI_IF
//todo 使用I2C接口
// #define USE_I2C_IF
//使用中断式获取数据(无latency),不定义则使用手动式获取数据
#define USE_INT_GET_DATA




#ifdef USE_SPI_IF
    //PC0 -> ICM_CS
    #define SPI_CS_CLK_EN    __GPIOA_CLK_ENABLE
    #define SPI_CS_PORT      ICM_CS_GPIO_Port
    #define SPI_CS_PIN       ICM_CS_Pin
    #include "sBSP_SPI.h"
#endif






sDRV_ICM45686_Data_t g_icm45686;
sDRV_ICM45686_Conf_t g_icm45686_conf;

//寄存器地址和掩码
/*Bank0的寄存器*/

//ACCEL_DATA_X1_UI 
#define ADDR_ACCEL_DATA_X1_UI                                        (0x00U)

//ACCEL_DATA_X0_UI
#define ADDR_ACCEL_DATA_X0_UI                                        (0x01U)

//ACCEL_DATA_Y1_UI
#define ADDR_ACCEL_DATA_Y1_UI                                        (0x02U)

//ACCEL_DATA_Y0_UI
#define ADDR_ACCEL_DATA_Y0_UI                                        (0x03U)

//ACCEL_DATA_Z1_UI
#define ADDR_ACCEL_DATA_Z1_UI                                        (0x04U)

//ACCEL_DATA_Z0_UI
#define ADDR_ACCEL_DATA_Z0_UI                                        (0x05U)

//GYRO_DATA_X1_UI
#define ADDR_GYRO_DATA_X1_UI                                         (0x06U)

//GYRO_DATA_X0_UI
#define ADDR_GYRO_DATA_X0_UI                                         (0x07U)

//GYRO_DATA_Y1_UI
#define ADDR_GYRO_DATA_Y1_UI                                         (0x08U)

//GYRO_DATA_Y0_UI
#define ADDR_GYRO_DATA_Y0_UI                                         (0x09U)

//GYRO_DATA_Z1_UI
#define ADDR_GYRO_DATA_Z1_UI                                         (0x0AU)

//GYRO_DATA_Z0_UI
#define ADDR_GYRO_DATA_Z0_UI                                         (0x0BU)

//TEMP_DATA1_UI
#define ADDR_TEMP_DATA1_UI                                           (0x0CU)

//TEMP_DATA1_UI
#define ADDR_TEMP_DATA0_UI                                           (0x0DU)

//TMST_FSYNCH
#define ADDR_TMSTFSYNCH                                              (0x0EU)

//TMST_FSYNCL
#define ADDR_TMSTFSYNCL                                              (0x0FU)

//PWR_MGMT0 电源管理
#define ADDR_PWR_MGMT0                                               (0x10U)
#define MSK_PWR_MGMT0_GYRO_MODE                                      (0b00001100U)
#define MSK_PWR_MGMT0_ACCEL_MODE                                     (0b00000011U)

//FIFO_COUNTH FIFO计数0
#define ADDR_FIFO_COUNT_0                                            (0x12U)

//FIFO_COUNTH FIFO计数1
#define ADDR_FIFO_COUNT_1                                            (0x13U)

//FIFO_DATA FIFO数据读取 
#define ADDR_FIFO_DATA                                               (0x14U)

//INT1_CONFIG0
#define ADDR_INT1_CONFIG0                                            (0x16U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_RESET_DONE                   (0b10000000U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_AUX1_AGC_RDY                 (0b01000000U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_AP_AGC_RDY                   (0b00100000U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_AP_FSYNC                     (0b00010000U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_AUX1_DRDY                    (0b00001000U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_DRDY                         (0b00000100U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_FIFO_THS                     (0b00000010U)
#define MSK_INT1_CONFIG0_INT1_STATUS_EN_FIFO_FULL                    (0b00000001U)

//INT1_CONFIG1
#define ADDR_INT1_CONFIG1                                            (0x17U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_APEX_EVENT                   (0b01000000U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_I2CM_DONE                    (0b00100000U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_I3C_PROTOCOL_ERR			 (0b00010000U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_WOM_Z						 (0b00001000U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_WOM_Y                        (0b00000100U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_WOM_X                        (0b00000010U)
#define MSK_INT1_CONFIG1_INT1_STATUS_EN_PLL_RDY                      (0B00000001U)

//INT1_CONFIG2
#define ADDR_INT1_CONFIG2                                            (0x18U)
#define MSK_INT1_CONFIG2_INT1_DRIVE                                  (0b00000100U)
#define MSK_INT1_CONFIG2_INT1_MODE                                   (0b00000010U)
#define MSK_INT1_CONFIG2_INT1_POLARITY                               (0b00000001U)

//INT1_STATUS0
#define ADDR_INT1_STATUS0                                            (0x19U)
#define MSK_INT1_STATUS0_INT1_STATUS_RESET_DONE                      (0b10000000U)
#define MSK_INT1_STATUS0_INT1_STATUS_AUX1_AGC_RDY                    (0b01000000U)
#define MSK_INT1_STATUS0_INT1_STATUS_AP_AGC_RDY                      (0b00100000U)
#define MSK_INT1_STATUS0_INT1_STATUS_AP_FSYNC                        (0b00010000U)
#define MSK_INT1_STATUS0_INT1_STATUS_AUX1_DRDY                       (0b00001000U)
#define MSK_INT1_STATUS0_INT1_STATUS_DRDY                            (0b00000100U)
#define MSK_INT1_STATUS0_INT1_STATUS_FIFO_THS                        (0b00000010U)
#define MSK_INT1_STATUS0_INT1_STATUS_FIFO_FULL                       (0b00000001U)

//INT1_STATUS1
#define ADDR_INT1_STATUS1                                            (0x1AU)
#define MSK_INT1_STATUS1_INT1_STATUS_APEX_EVENT                      (0b01000000U)
#define MSK_INT1_STATUS1_INT1_STATUS_I2CM_DONE                       (0b00100000U)
#define MSK_INT1_STATUS1_INT1_STATUS_I3C_PROTOCOL_ERR                (0b00010000U)
#define MSK_INT1_STATUS1_INT1_STATUS_WOM_Z                           (0b00001000U)
#define MSK_INT1_STATUS1_INT1_STATUS_WOM_Y                           (0b00000100U)
#define MSK_INT1_STATUS1_INT1_STATUS_WOM_X                           (0b00000010U)
#define MSK_INT1_STATUS1_INT1_STATUS_PLL_RDY                         (0b00000001U)

//ACCEL_CONFIG0
#define ADDR_ACCEL_CONFIG0                                           (0x1BU)
#define MSK_ACCEL_CONFIG0_ACCEL_UI_FS_SEL                            (0b01110000U)
#define MSK_ACCEL_CONFIG0_ACCEL_ODR                                  (0b00001111U)

//GYRO_CONFIG0
#define ADDR_GYRO_CONFIG0                                            (0x1CU)
#define MSK_GYRO_CONFIG0_GYRO_UI_FS_SEL                              (0b11110000U)
#define MSK_GYRO_CONFIG0_GYRO_ODR                                    (0b00001111U)

//FIFO_CONFIG0
#define ADDR_FIFO_CONFIG0                                            (0x1DU)
#define FIFO_CONFIG0FIFO_MODE                                        (0b11000000U)
#define MSK_FIFO_CONFIG0FIFO_DEPTH                                   (0b00111111U)

//FIFO_CONFIG1_0
#define ADDR_FIFO_CONFIG1_0                                          (0x1EU)

//FIFO_CONFIG1_1
#define ADDR_FIFO_CONFIG1_1                                          (0x1FU)

//FIFO_CONFIG2
#define ADDR_FIFO_CONFIG2                                            (0x20U)
#define MSK_FIFO_CONFIG2_FIFO_FLUSH                                  (0b10000000U)
#define MSK_FIFO_CONFIG2_FIFO_WR_WM_GT_TH                            (0b00001000U)

//FIFO_CONFIG3
#define ADDR_FIFO_CONFIG3                                            (0x21U)
#define MSK_FIFO_CONFIG3_FIFO_ES1_EN                                 (0b00100000U)
#define MSK_FIFO_CONFIG3_FIFO_ES0_EN                                 (0b00010000U)
#define MSK_FIFO_CONFIG3_FIFO_HIRES_EN                               (0b00001000U)
#define MSK_FIFO_CONFIG3_FIFO_GYRO_EN                                (0b00000100U)
#define MSK_FIFO_CONFIG3_FIFO_ACCEL_EN                               (0b00000010U)
#define MSK_FIFO_CONFIG3_FIFO_IF_EN                                  (0b00000001U)

//FIFO_CONFIG4
#define ADDR_FIFO_CONFIG4                                            (0x22U)
#define MSK_FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG                       (0b00111000U)
#define MSK_FIFO_CONFIG4_FIFO_COMP_EN                                (0b00000100U)
#define MSK_FIFO_CONFIG4_FIFO_TMST_FSYNC_EN                          (0b00000010U)
#define MSK_FIFO_CONFIG4_FIFO_ES0_6B_9B                              (0b00000001U)

//TMST_WOM_CONFIG
#define ADDR_TMST_WOM_CONFIG                                         (0x23U)
#define MSK_TMST_WOM_CONFIG_TMST_DELTA_EN                            (0b01000000U)
#define MSK_TMST_WOM_CONFIG_TMST_RESOL                               (0b00100000U)
#define MSK_TMST_WOM_CONFIG_WOM_EN                                   (0b00010000U)
#define MSK_TMST_WOM_CONFIG_WOM_MODE                                 (0b00001000U)
#define MSK_TMST_WOM_CONFIG_WOM_INT_MODE                             (0b00000100U)
#define MSK_TMST_WOM_CONFIG_WOM_INT_DUR                              (0b00000011U)

//FSYNC_CONFIG0
#define ADDR_FSYNC_CONFIG0                                           (0x24U)
#define MSK_FSYNC_CONFIG0_AP_FSYNC_FLAG_CLEAR_SEL                    (0b00001000U)
#define MSK_FSYNC_CONFIG0_AP_FSYNC_SEL                               (0b00000111U)

//FSYNC_CONFIG1
#define ADDR_FSYNC_CONFIG1                                           (0x25U)
#define MSK_FSYNC_CONFIG1_AUX1_FSYNC_FLAG_CLEAR_SEL                  (0b00001000U)
#define MSK_FSYNC_CONFIG1_AUX1_FSYNC_SEL                             (0b00000111U)

//RTC_CONFIG
#define ADDR_RTC_CONFIG                                              (0x26U)
#define MSK_RTC_CONFIG_RTC_ALIGN                                     (0b01000000U)
#define MSK_RTC_CONFIG_RTC_MODE                                      (0b00100000U)

//DMP_EXT_SEN_ODR_CFG
#define ADDR_DMP_EXT_SEN_ODR_CFG                                     (0x27U)
#define MSK_DMP_EXT_SEN_ODR_CFGEXT_SENSOR_EN                         (0b01000000U)
#define MSK_DMP_EXT_SEN_ODR_CFGEXT_ODR                               (0b00111000U)
#define MSK_DMP_EXT_SEN_ODR_CFGAPEX_ODR                              (0b00000111U)

//ODR_DECIMATE_CONFIG
#define ADDR_ODR_DECIMATE_CONFIG                                     (0x28U)
#define MSK_ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC                    (0b11110000U)
#define MSK_ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC                   (0b00001111U)

//EDMP_APEX_EN0
#define ADDR_EDMP_APEX_EN0                                           (0x29U)
#define MSK_EDMP_APEX_EN0_SMD_EN                                     (0b10000000U)
#define MSK_EDMP_APEX_EN0_R2W_EN                                     (0b01000000U)
#define MSK_EDMP_APEX_EN0_FF_EN                                      (0b00100000U)
#define MSK_EDMP_APEX_EN0_PEDO_EN                                    (0b00010000U)
#define MSK_EDMP_APEX_EN0_TILT_EN                                    (0b00001000U)
#define MSK_EDMP_APEX_EN0_TAP_EN                                     (0b00000001U)

//EDMP_APEX_EN1
#define ADDR_EDMP_APEX_EN1                                           (0x2AU)
#define MSK_EDMP_APEX_EN1_EDMP_ENABLE                                (0b01000000U)
#define MSK_EDMP_APEX_EN1_FEATURE3_EN                                (0b00100000U)
#define MSK_EDMP_APEX_EN1_POWER_SAVE_EN                              (0b00000100U)
#define MSK_EDMP_APEX_EN1_INIT_EN                                    (0b00000010U)
#define MSK_EDMP_APEX_EN1_SOFT_HARD_IRON_CORR_EN                     (0b00000001U)

//APEX_BUFFER_MGMT
#define ADDR_APEX_BUFFER_MGMT                                        (0x2BU)
#define MSK_APEX_BUFFER_MGMT_FF_DURATION_HOST_RPTR                   (0b11000000U)
#define MSK_APEX_BUFFER_MGMT_FF_DURATION_EDMP_WPTR                   (0b00110000U)
#define MSK_APEX_BUFFER_MGMT_STEP_COUNT_HOST_RPTR                    (0b00001100U)
#define MSK_APEX_BUFFER_MGMT_STEP_COUNT_EDMP_WPTR                    (0b00000011U)

//INTF_CONFIG0
#define ADDR_INTF_CONFIG0                                            (0x2CU)
#define MSK_INTF_CONFIG0_VIRTUAL_ACCESS_AUX1_EN                      (0b00100000U)
#define MSK_INTF_CONFIG0_AP_SPI_34_MODE                              (0b00000010U)
#define MSK_INTF_CONFIG0_AP_SPI_MODE                                 (0b00000001U)

//INTF_CONFIG1_OVRD
#define ADDR_INTF_CONFIG1_OVRD                                       (0x2DU)
#define MSK_INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD                    (0b00001000U)
#define MSK_INTF_CONFIG1_OVRD_AP_SPI_34_MODE_OVRD_VAL                (0b00000100U)
#define MSK_INTF_CONFIG1_OVRD_AP_SPI_MODE_OVRD                       (0b00000010U)
#define MSK_INTF_CONFIG1_OVRD_AP_SPI_MODE_OVRD_VAL                   (0b00000001U)

//INTF_AUX_CONFIG
#define ADDR_INTF_AUX_CONFIG                                         (0x2EU)
#define MSK_INTF_AUX_CONFIG_AUX1_SPI_34_MODE                         (0b00000010U)
#define MSK_INTF_AUX_CONFIG_AUX1_SPI_MODE                            (0b00000001U)

//IOC_PAD_SCENARIO
#define ADDR_IOC_PAD_SCENARIO                                        (0x2FU)
#define MSK_IOC_PAD_SCENARIO_AUX1_MODE                               (0b00000110U)
#define MSK_IOC_PAD_SCENARIO_AUX1_ENABLE                             (0b00000001U)

//IOC_PAD_SCENARIO_AUX_OVRD
#define ADDR_IOC_PAD_SCENARIO_AUX_OVRD                               (0x30U)
#define MSK_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_MODE_OVRD                 (0b00010000U)
#define MSK_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_ENABLE_OVRD_VAL           (0b00001100U)
#define MSK_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_ENABLE_OVRD               (0b00000010U)
//! 这里好像有两个一样的位段
// #define MSK_IOC_PAD_SCENARIO_AUX_OVRD_AUX1_ENABLE_OVRD_VAL           (0b00000001U)

//DRIVE_CONFIG0
#define ADDR_DRIVE_CONFIG0                                           (0x32U)
#define MSK_DRIVE_CONFIG0_PADS_I2C_SLEW                              (0b01110000U)
#define MSK_DRIVE_CONFIG0_PADS_SPI_SLEW                              (0b00001110U)

//DRIVE_CONFIG1
#define ADDR_DRIVE_CONFIG1                                           (0x33U)
#define MSK_DRIVE_CONFIG1_PADS_I3C_DDR_SLEW                          (0b00111000U)
#define MSK_DRIVE_CONFIG1_PADS_I3C_SDR_SLEW                          (0b00000111U)

//DRIVE_CONFIG2
#define ADDR_DRIVE_CONFIG2                                           (0x34U)
#define MSK_DRIVE_CONFIG2_PADS_SLEW                                  (0b00000111U)

//REG_MISC1
#define ADDR_REG_MISC1                                               (0x35U)
#define MSK_REG_MISC1_OSC_ID_OVRD                                    (0b00001111U)

//INT_APEX_CONFIG0
#define ADDR_INT_APEX_CONFIG0                                        (0x39U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_R2W_WAKE_DET        (0b10000000U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_FF_DET              (0b01000000U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_STEP_DET            (0b00100000U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_STEP_CNT_OVFL       (0b00010000U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_TILT_DET            (0b00001000U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_LOW_G_DET           (0b00000100U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_HIGH_G_DET          (0b00000010U)
#define MSK_INT_APEX_CONFIG0_INT_STATUS_MASK_PIN_TAP_DET             (0b00000001U)

//INT_APEX_CONFIG1
#define ADDR_INT_APEX_CONFIG1                                        (0x3AU)
#define MSK_INT_APEX_CONFIG1_INT_STATUS_MASK_PIN_SA_DONE             (0b00010000U)
#define MSK_INT_APEX_CONFIG1_INT_STATUS_MASK_PIN_SELFTEST_DONE       (0b00000100U)
#define MSK_INT_APEX_CONFIG1_INT_STATUS_MASK_PIN_SMD_DET             (0b00000010U)
#define MSK_INT_APEX_CONFIG1_INT_STATUS_MASK_PIN_R2W_SLEEP_DET       (0b00000001U)

//INT_APEX_STATUS0
#define ADDR_INT_APEX_STATUS0                                        (0x3BU)
#define MSK_INT_APEX_STATUS0_INT_STATUS_R2W_WAKE_DET                 (0b10000000U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_FF_DET                       (0b01000000U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_STEP_DET                     (0b00100000U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_STEP_CNT_OVFL                (0b00010000U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_TILT_DET                     (0b00001000U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_LOW_G_DET                    (0b00000100U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_HIGH_G_DET                   (0b00000010U)
#define MSK_INT_APEX_STATUS0_INT_STATUS_TAP_DET                      (0b00000001U)

//INT_APEX_STATUS1
#define ADDR_INT_APEX_STATUS1                                        (0x3CU)
#define MSK_INT_APEX_STATUS1_INT_STATUS_SA_DONE                      (0b00010000U)
#define MSK_INT_APEX_STATUS1_INT_STATUS_SELFTEST_DONE                (0b00000100U)
#define MSK_INT_APEX_STATUS1_INT_STATUS_SMD_DET                      (0b00000010U)
#define MSK_INT_APEX_STATUS1_INT_STATUS_R2W_SLEEP_DET                (0b00000001U)

//ACCEL_DATA_X1_AUX1  
#define ADDR_ACCEL_DATA_X1_AUX1                                      (0x44U)

//ACCEL_DATA_X0_AUX1  
#define ADDR_ACCEL_DATA_X0_AUX1                                      (0x45U)

//ACCEL_DATA_Y1_AUX1  
#define ADDR_ACCEL_DATA_Y1_AUX1                                      (0x46U)

//ACCEL_DATA_Y0_AUX1  
#define ADDR_ACCEL_DATA_Y0_AUX1                                      (0x47U)

//ACCEL_DATA_Z1_AUX1  
#define ADDR_ACCEL_DATA_Z1_AUX1                                      (0x48U)

//ACCEL_DATA_Z0_AUX1  
#define ADDR_ACCEL_DATA_Z0_AUX1                                      (0x49U)

//GYRO_DATA_X1_AUX1  
#define ADDR_GYRO_DATA_X1_AUX1                                       (0x4AU)

//DATA_X0_AUX1  
#define ADDR__DATA_X0_AUX1                                           (0x4BU)

//GYRO_DATA_Y1_AUX1  
#define ADDR_GYRO_DATA_Y1_AUX1                                       (0x4CU)

//GYRO_DATA_Y0_AUX1  
#define ADDR_GYRO_DATA_Y0_AUX1                                       (0x4DU)

//GYRO_DATA_Z1_AUX1  
#define ADDR_GYRO_DATA_Z1_AUX1                                       (0x4EU)

//GYRO_DATA_Z0_AUX1  
#define ADDR_GYRO_DATA_Z0_AUX1                                       (0x4FU)

//TEMP_DATA1_AUX1  
#define ADDR_TEMP_DATA1_AUX1                                         (0x50U)

//TEMP_DATA0_AUX1  
#define ADDR_TEMP_DATA0_AUX1                                         (0x51U)

//TMST_FSYNCH_AUX1  
#define ADDR_TMST_FSYNCH_AUX1                                        (0x52U)

//TMST_FSYNCL_AUX1  
#define ADDR_TMST_FSYNCL_AUX1                                        (0x53U)

//PWR_MGMT_AUX1  
#define ADDR_PWR_MGMT_AUX1                                           (0x54U)
#define MSK_PWR_MGMT_AUX1_GYRO_AUX1_EN                               (0b00000010U)
#define MSK_PWR_MGMT_AUX1_ACCEL_AUX1_EN                              (0b00000001U)

//FS_SEL_AUX1  
#define ADDR_FS_SEL_AUX1                                             (0x55U)
#define MSK_FS_SEL_AUX1_GYRO_AUX1_FS_SEL                             (0b01111000U)
#define MSK_FS_SEL_AUX1_ACCEL_AUX1_FS_SEL                            (0b00000111U)

//INT2_CONFIG0  
#define ADDR_INT2_CONFIG0                                            (0x56U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_RESET_DONE                   (0b10000000U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_AUX1_AGC_RDY                 (0b01000000U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_AP_AGC_RDY                   (0b00100000U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_AP_FSYNC                     (0b00010000U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_AUX1_DRDY                    (0b00001000U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_DRDY                         (0b00000100U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_FIFO_THS                     (0b00000010U)
#define MSK_INT2_CONFIG0_INT2_STATUS_EN_FIFO_FULL                    (0b00000001U)

//INT2_CONFIG1  
#define ADDR_INT2_CONFIG1                                            (0x57U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_APEX_EVENT                   (0b01000000U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_I2CM_DONE                    (0b00100000U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_I3C_PROTOCOL_ERR             (0b00010000U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_WOM_Z                        (0b00001000U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_WOM_Y                        (0b00000100U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_WOM_X                        (0b00000010U)
#define MSK_INT2_CONFIG1_INT2_STATUS_EN_PLL_RDY                      (0b00000001U)

//INT2_CONFIG2  
#define ADDR_INT2_CONFIG2                                            (0x58U)
#define MSK_INT2_CONFIG2_INT2_DRIVE                                  (0b00000100U)
#define MSK_INT2_CONFIG2_INT2_MODE                                   (0b00000010U)
#define MSK_INT2_CONFIG2_INT2_POLARITY                               (0b00000001U)

//INT2_STATUS0  
#define ADDR_INT2_STATUS0                                            (0x59U)
#define MSK_INT2_STATUS0_INT2_STATUS_RESET_DONE                      (0b10000000U)
#define MSK_INT2_STATUS0_INT2_STATUS_AUX1_AGC_RDY                    (0b01000000U)
#define MSK_INT2_STATUS0_INT2_STATUS_AP_AGC_RDY                      (0b00100000U)
#define MSK_INT2_STATUS0_INT2_STATUS_AP_FSYNC                        (0b00010000U)
#define MSK_INT2_STATUS0_INT2_STATUS_AUX1_DRDY                       (0b00001000U)
#define MSK_INT2_STATUS0_INT2_STATUS_DRDY                            (0b00000100U)
#define MSK_INT2_STATUS0_INT2_STATUS_FIFO_THS                        (0b00000010U)
#define MSK_INT2_STATUS0_INT2_STATUS_FIFO_FULL                       (0b00000001U)

//INT2_STATUS1  
#define ADDR_INT2_STATUS1                                            (0x5AU)
#define MSK_INT2_STATUS1_INT1_STATUS_APEX_EVENT                      (0b01000000U)
#define MSK_INT2_STATUS1_INT1_STATUS_I2CM_DONE                       (0b00100000U)
#define MSK_INT2_STATUS1_INT1_STATUS_I3C_PROTOCOL_ERR                (0b00010000U)
#define MSK_INT2_STATUS1_INT1_STATUS_WOM_Z                           (0b00001000U)
#define MSK_INT2_STATUS1_INT1_STATUS_WOM_Y                           (0b00000100U)
#define MSK_INT2_STATUS1_INT1_STATUS_WOM_X                           (0b00000010U)
#define MSK_INT2_STATUS1_INT1_STATUS_PLL_RDY                         (0b00000001U)

//WHO_AM_I  
#define ADDR_WHO_AM_I                                                (0x72U)
#define MSK_WHO_AM_I_WHOAMI                                          (0b11111111U)

//REG_HOST_MSG  
#define ADDR_REG_HOST_MSG                                            (0x73U)
#define MSK_REG_HOST_MSG_EDMP_ON_DEMAND_EN                           (0b00100000U)
#define MSK_REG_HOST_MSG_TESTOPENABLE                                (0b00000001U)

//IREG_ADDR_15_8  
#define ADDR_IREG_ADDR_15_8                                          (0x7CU)

//IREG_ADDR_7_0  
#define ADDR_IREG_ADDR_7_0                                           (0x7DU)

//IREG_DATA  
#define ADDR_IREG_DATA                                               (0x7EU)                

//REG_MISC2  
#define ADDR_REG_MISC2                                               (0x7FU)
#define MSK_REG_MISC2_SOFT_RST                                       (0b00000010U)
#define MSK_REG_MISC2_IREG_DONE                                      (0b00000001U)


//接口
//初始化CS引脚
static void portCSInit(){
    SPI_CS_CLK_EN();
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Pin   = SPI_CS_PIN;
    HAL_GPIO_Init(SPI_CS_PORT,&gpio);
}

static inline void portSetCS(bool lv){
    HAL_GPIO_WritePin(SPI_CS_PORT,SPI_CS_PIN,(GPIO_PinState)lv);
}

#ifdef USE_INT_GET_DATA
static void portINTInit(){
    sBSP_GPIO_IcmInt_Init();
}
#endif




static uint8_t read_reg(uint8_t addr){
    //读时序有两个字节,第一个字节最高位为1表示读,为0表示写时序,其他位为寄存器地址
    portSetCS(0);
    //首先发送第一个字节
    sBSP_SPI_IMU_SendByte(0x80 | addr);
    //然后读取第二个字节
    uint8_t tmp = sBSP_SPI_IMU_RecvByte();
    portSetCS(1);
    return tmp;
}
static void write_reg(uint8_t addr,uint8_t data){
    portSetCS(0);
    //首先发送第一个字节
    sBSP_SPI_IMU_SendByte(0x00 | addr);
    //然后发送第二个字节
    sBSP_SPI_IMU_SendByte(data);
    portSetCS(1);
}
//一次读取多个寄存器,用于加速读取原始数据速度
static void read_regs(uint8_t addr,uint8_t* pData,uint8_t len){
    portSetCS(0);
    //首先发送第一个字节
    sBSP_SPI_IMU_SendByte(0x80 | addr);
    //然后读取
    sBSP_SPI_IMU_RecvBytes(pData,len);
    portSetCS(1);
}


#include "sUtils.h"


// 对寄存器进行修改,形参:寄存器地址,修改的部分,修改的数据
static void reg_modify(uint8_t reg_addr, uint8_t reg_msk, uint8_t data){
    //读改写
    //首先读出寄存器
    uint8_t tmpreg = read_reg(reg_addr);
	// sBSP_UART_Debug_Printf("读出位置0x%X的内容:0x%X\n",reg_addr,tmpreg);
    //进行修改
    sut_modify_reg(&tmpreg, reg_msk, data);
	// sBSP_UART_Debug_Printf("位掩码:0x%X,数据:0x%X,修改后的内容:0x%X\n\n",reg_msk,data,tmpreg);
    //写回
    write_reg(reg_addr, tmpreg);
}




int sDRV_ICM45686_Init(){
    #ifdef USE_SPI_IF
        portCSInit();
        portSetCS(1);
    #endif
    
    //检查通信是否正常
    uint8_t who_am_i = read_reg(ADDR_WHO_AM_I);
    if(who_am_i != 0xE9){
        // return who_am_i;
        return -1;
    }

    #ifdef USE_INT_GET_DATA
        portINTInit();
        //使能INT1数据准备好了中断
        reg_modify(ADDR_INT1_CONFIG0,MSK_INT1_CONFIG0_INT1_STATUS_EN_DRDY,1);
        //开漏模式
        reg_modify(ADDR_INT1_CONFIG2,MSK_INT1_CONFIG2_INT1_DRIVE,1);
        //中断PULSE模式
        reg_modify(ADDR_INT1_CONFIG2,MSK_INT1_CONFIG2_INT1_MODE,0);
        //中断激活为LOW
        reg_modify(ADDR_INT1_CONFIG2,MSK_INT1_CONFIG2_INT1_POLARITY,0);
    #endif

    //初始化配置:实时性+++
    sDRV_ICM45686_Conf_t icm_conf = {0};
    icm_conf.gyro_fs    = SDRV_ICM45686_GYRO_UI_FS_SEL_1000DPS;
    icm_conf.gyro_odr   = SDRV_ICM45686_GYRO_ODR_800HZ;
    // icm_conf.gyro_odr   = SDRV_ICM45686_GYRO_ODR_400HZ;

    icm_conf.accel_fs   = SDRV_ICM45686_ACCEL_UI_FS_SEL_4G;
    icm_conf.accel_odr  = SDRV_ICM45686_ACCEL_ODR_50HZ;
    icm_conf.gyro_mode  = SDRV_ICM45686_GYRO_MODE_LN;
    icm_conf.accel_mode = SDRV_ICM45686_ACCEL_MODE_LN;
    sDRV_ICM45686_SetConfig(&icm_conf); 

    return 0;
}


void sDRV_ICM45686_SetConfig(const sDRV_ICM45686_Conf_t* p_conf){
    //如果传入NULL则更新本地的配置结构体,否则更新为用户传入的配置
    if(p_conf != NULL){
        memcpy(&g_icm45686_conf,p_conf,sizeof(sDRV_ICM45686_Conf_t));
    }
    
    //bank=0,上传配置
    // reg_modify(ADDR_REGBANKSEL,MSK_REGBANKSEL_BANKSEL,SDRV_ICM_REGBANKSEL_BANK0);
    
    reg_modify(ADDR_GYRO_CONFIG0 ,MSK_GYRO_CONFIG0_GYRO_UI_FS_SEL  ,g_icm45686_conf.gyro_fs);
    reg_modify(ADDR_GYRO_CONFIG0 ,MSK_GYRO_CONFIG0_GYRO_ODR        ,g_icm45686_conf.gyro_odr);
    reg_modify(ADDR_ACCEL_CONFIG0,MSK_ACCEL_CONFIG0_ACCEL_UI_FS_SEL,g_icm45686_conf.accel_fs);
    reg_modify(ADDR_ACCEL_CONFIG0,MSK_ACCEL_CONFIG0_ACCEL_ODR      ,g_icm45686_conf.accel_odr);
    reg_modify(ADDR_PWR_MGMT0    ,MSK_PWR_MGMT0_GYRO_MODE          ,g_icm45686_conf.gyro_mode);
    reg_modify(ADDR_PWR_MGMT0    ,MSK_PWR_MGMT0_ACCEL_MODE         ,g_icm45686_conf.accel_mode);
}





void sDRV_ICM45686_GetData(){
    uint8_t buf[14];
    read_regs(ADDR_ACCEL_DATA_X1_UI,buf,14);

    g_icm45686.temp  = (float)((int16_t)((buf[13] << 8) | buf[12]) / 128 + 25);
    g_icm45686.acc_x = (float)(int16_t)((uint16_t)(buf[ 1] << 8) | (uint16_t)buf[ 0]);
    g_icm45686.acc_y = (float)(int16_t)((uint16_t)(buf[ 3] << 8) | (uint16_t)buf[ 2]);
	g_icm45686.acc_z = (float)(int16_t)((uint16_t)(buf[ 5] << 8) | (uint16_t)buf[ 4]);
	g_icm45686.gyr_x = (float)(int16_t)((uint16_t)(buf[ 7] << 8) | (uint16_t)buf[ 6]);
	g_icm45686.gyr_y = (float)(int16_t)((uint16_t)(buf[ 9] << 8) | (uint16_t)buf[ 8]);
    g_icm45686.gyr_z = (float)(int16_t)((uint16_t)(buf[11] << 8) | (uint16_t)buf[10]);

    //处理数据
	//处理加速度,单位m/s^2
	if(g_icm45686_conf.accel_fs == SDRV_ICM45686_ACCEL_UI_FS_SEL_2G){
		g_icm45686.acc_x *= ( 2.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_y *= ( 2.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_z *= ( 2.0f / 32768.0f) * M_GRAVITY;
	}else if(g_icm45686_conf.accel_fs == SDRV_ICM45686_ACCEL_UI_FS_SEL_4G){
		g_icm45686.acc_x *= ( 4.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_y *= ( 4.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_z *= ( 4.0f / 32768.0f) * M_GRAVITY;
	}else if(g_icm45686_conf.accel_fs == SDRV_ICM45686_ACCEL_UI_FS_SEL_8G){
		g_icm45686.acc_x *= ( 8.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_y *= ( 8.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_z *= ( 8.0f / 32768.0f) * M_GRAVITY;
	}else if(g_icm45686_conf.accel_fs == SDRV_ICM45686_ACCEL_UI_FS_SEL_16G){
		g_icm45686.acc_x *= (16.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_y *= (16.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_z *= (16.0f / 32768.0f) * M_GRAVITY;
	}else if(g_icm45686_conf.accel_fs == SDRV_ICM45686_ACCEL_UI_FS_SEL_32G){
		g_icm45686.acc_x *= (32.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_y *= (32.0f / 32768.0f) * M_GRAVITY;
        g_icm45686.acc_z *= (32.0f / 32768.0f) * M_GRAVITY;
	}

    //处理角速度
	if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_250DPS){
		g_icm45686.gyr_x *= ( 250.0f / 32768.0f);
        g_icm45686.gyr_y *= ( 250.0f / 32768.0f);
        g_icm45686.gyr_z *= ( 250.0f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_500DPS){
		g_icm45686.gyr_x *= ( 500.0f / 32768.0f);
        g_icm45686.gyr_y *= ( 500.0f / 32768.0f);
        g_icm45686.gyr_z *= ( 500.0f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_1000DPS){
		g_icm45686.gyr_x *= (1000.0f / 32768.0f);
        g_icm45686.gyr_y *= (1000.0f / 32768.0f);
        g_icm45686.gyr_z *= (1000.0f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_2000DPS){
		g_icm45686.gyr_x *= (2000.0f / 32768.0f);
        g_icm45686.gyr_y *= (2000.0f / 32768.0f);
        g_icm45686.gyr_z *= (2000.0f / 32768.0f);
    }else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_4000DPS){
		g_icm45686.gyr_x *= (4000.0f / 32768.0f);
        g_icm45686.gyr_y *= (4000.0f / 32768.0f);
        g_icm45686.gyr_z *= (4000.0f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_125DPS){
		g_icm45686.gyr_x *= ( 125.0f / 32768.0f);
        g_icm45686.gyr_y *= ( 125.0f / 32768.0f);
        g_icm45686.gyr_z *= ( 125.0f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_62D5DPS){
		g_icm45686.gyr_x *= (  62.5f / 32768.0f);
        g_icm45686.gyr_y *= (  62.5f / 32768.0f);
        g_icm45686.gyr_z *= (  62.5f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_31D25DPS){
		g_icm45686.gyr_x *= ( 31.25f / 32768.0f);
        g_icm45686.gyr_y *= ( 31.25f / 32768.0f);
        g_icm45686.gyr_z *= ( 31.25f / 32768.0f);
	}else if(g_icm45686_conf.gyro_fs == SDRV_ICM45686_GYRO_UI_FS_SEL_15D625DPS){
		g_icm45686.gyr_x *= (15.625f / 32768.0f);
        g_icm45686.gyr_y *= (15.625f / 32768.0f);
        g_icm45686.gyr_z *= (15.625f / 32768.0f);
	}
}



