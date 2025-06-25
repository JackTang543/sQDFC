#include "sDRV_SPL06.h"



#include "sBSP_I2C.h"


#define SPL06_I2C_ADDR            (0x77 << 1) // SPL06的I2C地址

#define REG_PSR_B2                0x00
#define REG_PSR_B1                0x01
#define REG_PSR_B0                0x02
#define REG_TMP_B2                0x03
#define REG_TMP_B1                0x04
#define REG_TMP_B0                0x05

#define REG_PRS_CFG               0x06
#define MSK_PRS_CFG_PM_RATE       0b01110000
#define MSK_PRS_CFG_PM_PRCS       0b00001111

#define REG_TMP_CFG               0x07
#define MSK_TMP_CFG_TMP_EXT       0b10000000
#define MSK_TMP_CFG_TMP_RATE      0b01110000
#define MSK_TMP_CFG_TMP_PRCS      0b00001111

#define REG_MEAS_CFG              0x08
#define MSK_MEAS_CFG_COEF_RDY     0b10000000
#define MSK_MEAS_CFG_SENSOR_RDY   0b01000000
#define MSK_MEAS_CFG_TMP_RDY      0b00100000
#define MSK_MEAS_CFG_PRS_RDY      0b00010000
#define MSK_MEAS_CFG_MEAS_CRTL    0b00000111

#define REG_CFG_REG               0x09
#define MSK_CFG_REG_INT_HL        0b10000000
#define MSK_CFG_REG_INT_SEL       0b01110000
#define MSK_CFG_REG_TMP_SHIFT_EN  0b00001000
#define MSK_CFG_REG_PRS_SHIFT_EN  0b00000100
#define MSK_CFG_REG_FIFO_EN       0b00000010

#define REG_INT_STS               0x0A
#define MSK_INT_STS_INT_FIFO_FULL 0b00000100
#define MSK_INT_STS_INT_TMP       0b00000010
#define MSK_INT_STS_INT_PRS       0b00000001

#define REG_FIFO_STS              0x0B
#define MSK_FIFO_STS_FIFO_FULL    0b00000010
#define MSK_FIFO_STS_FIFO_EMPTY   0b00000001

#define REG_RESET                 0x0C
#define MSK_RESET_FIFO_FLUSH      0b10000000
#define MSK_RESET_SOFT_RST        0b00001111

// ID
#define REG_ID                    0x0D
#define MSK_ID_PROD_ID            0b11110000
#define MSK_ID_REV_ID             0b00001111

// 从0x10到0x21
#define REG_COEF                  0x10


/*接口*/
static int read_reg(uint8_t reg,uint8_t* data){
    return sBSP_I2C1M_MemReadBytes(SPL06_I2C_ADDR,reg,I2C_MEMADD_SIZE_8BIT,data,1);
}

static int read_regs(uint8_t reg, uint8_t *data, uint8_t len) {
    return sBSP_I2C1M_MemReadBytes(SPL06_I2C_ADDR,reg, I2C_MEMADD_SIZE_8BIT, data, len);
}

static int write_reg(uint8_t reg, uint8_t data){
    return sBSP_I2C1M_MemSendBytes(SPL06_I2C_ADDR,reg,I2C_MEMADD_SIZE_8BIT,&data,1);
}

static int write_regs(uint8_t reg, uint8_t *data, uint8_t len) {
    return sBSP_I2C1M_MemSendBytes(SPL06_I2C_ADDR,reg, I2C_MEMADD_SIZE_8BIT, data, len);
}


static int send_reset(){
    uint8_t data = 0b00001001;
    return write_reg(REG_RESET, data);
}





#include "sUtils.h"


int sDRV_SPL06_Init() {
    //读取ID
    uint8_t id = 0;

    read_reg(REG_ID, &id);
    if(id != 0x10){
        log_error("SPL06 ID error: %02X", id);
        return -1;
    }

    // if(send_reset() != 0){
    //     log_error("SPL06 reset error");
    //     return -2;
    // }

    //写PRS_CFG为0x77 128Hz,over sampling 128
    write_reg(REG_PRS_CFG, 0x77);

    //写TMP_CFG为0x23 INT,16Hz,over sampling 8
    write_reg(REG_TMP_CFG, 0x23);

    //写MEAS_CFG为0x07


    HAL_Delay(100);







    return 0;
}



int sDRV_SPL06_ReadData(float* press_pa,float* temp_c){
    // uint8_t data[6] = {0};
    // if(read_reg(REG_PSR_B2, data, 6) != 0){
    //     log_error("SPL06 read data error");
    //     return -1;
    // }

    int flag = -1;

    //每个寄存器单独读取
    uint8_t data[6] = {0};
    flag |= read_reg(REG_PSR_B2, &data[0]);
    flag |= read_reg(REG_PSR_B1, &data[1]);
    flag |= read_reg(REG_PSR_B0, &data[2]);
    flag |= read_reg(REG_TMP_B2, &data[3]);
    flag |= read_reg(REG_TMP_B1, &data[4]);
    flag |= read_reg(REG_TMP_B0, &data[5]);
    // uint32_t press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    // uint32_t temp = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    //合并3个byte,b2为最高字节,b1为中间字节,b0为最低字节
    uint32_t press = (data[0] << 16) | (data[1] << 8) | data[2];
    uint32_t temp = (data[3] << 16) | (data[4] << 8) | data[5];


    *press_pa = press / 4096.0f;
    *temp_c = temp / 4096.0f;

    if(flag != 0){
        log_error("SPL06 read data error");
        return -1;
    }
    return 0;
}









