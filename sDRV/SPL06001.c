//#ifndef SPL06_01_C
//#define SPL06_01_C

#include "spl06001.h"

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val);
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr);
void spl0601_get_calib_param(void);

/*****************************************************************************
 Function: spl0601_write
 Description: this function will write data to specofic register through software I2C bus
 Input:  uint8 hwadr   hardware I2C address
         uint8 regadr  register address
         uint8 val     write-in value          
 Output: 
 Return: 
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_write(uint8_t hwadr, uint8_t regadr, uint8_t val)
{
  HAL_I2C_Mem_Write(&hi2c1,hwadr<<1,regadr,I2C_MEMADD_SIZE_8BIT,&val,1,100);
}

/*****************************************************************************
 Function: spl0601_read
 Description: this function will read register data through software I2C bus
 Input: uint8 hwadr   hardware I2C address
        uint8 regadr  register address        
 Output: 
 Return: uint8 readout value
 Calls: 
 Called By: 
*****************************************************************************/
uint8_t spl0601_read(uint8_t hwadr, uint8_t regadr)
{
    static uint8_t val = 0;
     HAL_I2C_Mem_Read(&hi2c1,hwadr<<1,regadr,I2C_MEMADD_SIZE_8BIT,&val,1,100);
    return val;
}

/*****************************************************************************
 Function: spl0601_init
 Description: initialization
 Input: void             
 Output: 
 Return: void 
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_init(void)
{   uint8_t temp;
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    HAL_I2C_Mem_Read(&hi2c1,(HW_ADR<<1),0x0d,I2C_MEMADD_SIZE_8BIT,&temp,1,HAL_MAX_DELAY);
    p_spl0601->chip_id = temp;
    spl0601_get_calib_param();
    // sampling rate = 4Hz; Pressure oversample = 32;
    spl0601_rateset(PRESSURE_SENSOR,4, 32);   
    // sampling rate = 4Hz; Temperature oversample = 32; 
    spl0601_rateset(TEMPERATURE_SENSOR,4, 32);
    //Start background measurement
    
}

/*****************************************************************************
 Function: spl0601_rateset
 Description: set sample rate and over sample rate per second for specific sensor
 Input:     uint8 u8OverSmpl  oversample rate         Maximal = 128
            uint8 u8SmplRate  sample rate(Hz) Maximal = 128
            uint8 iSensor     0: Pressure; 1: Temperature 
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

/*****************************************************************************
 Function: spl0601_get_calib_param
 Description: obtain the calibrated coefficient
 Input: void     
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32_t h;
    uint32_t m;
    uint32_t l;
    h =  spl0601_read(HW_ADR, 0x10);
    l  =  spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16_t)h<<4 | l>>4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0&0x0800)?(0xF000|p_spl0601->calib_param.c0):p_spl0601->calib_param.c0;
    h =  spl0601_read(HW_ADR, 0x11);
    l  =  spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16_t)(h&0x0F)<<8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1&0x0800)?(0xF000|p_spl0601->calib_param.c1):p_spl0601->calib_param.c1;
    h =  spl0601_read(HW_ADR, 0x13);
    m =  spl0601_read(HW_ADR, 0x14);
    l =  spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00&0x080000)?(0xFFF00000|p_spl0601->calib_param.c00):p_spl0601->calib_param.c00;
    h =  spl0601_read(HW_ADR, 0x15);
    m =  spl0601_read(HW_ADR, 0x16);
    l =  spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32_t)(h&0x0F)<<16 | (int32_t)m<<8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10&0x080000)?(0xFFF00000|p_spl0601->calib_param.c10):p_spl0601->calib_param.c10;
    h =  spl0601_read(HW_ADR, 0x18);
    l  =  spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1A);
    l  =  spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1C);
    l  =  spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x1E);
    l  =  spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16_t)h<<8 | l;
    h =  spl0601_read(HW_ADR, 0x20);
    l  =  spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16_t)h<<8 | l;
}

/*****************************************************************************
 Function: spl0601_start_temperature
 Description: start one measurement for temperature
 Input: void    
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 Function: spl0601_start_pressure
 Description: start one measurement for pressure
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}
/*****************************************************************************
 Function: spl0601_start_continuous
 Description: Select mode for the continuously measurement
 Input: uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature        
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_start_continuous(uint8_t mode)
{
    spl0601_write(HW_ADR, 0x08, mode+4);
}

void spl0601_stop(void)
{
    spl0601_write(HW_ADR, 0x08, 0);
}

/*****************************************************************************
 Function: spl0601_get_raw_temp
 Description:obtain the original temperature value and turn them into 32bits-integer 
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_temp(void)
{

    static uint8_t temp[3];
    HAL_I2C_Mem_Read(&hi2c1,HW_ADR<<1,0x03,I2C_MEMADD_SIZE_8BIT,temp,3,100);
    p_spl0601->i32rawTemperature = ((int32_t)temp[0]<<16 )| ((int32_t)temp[1]<<8 )| ((int32_t)temp[2]);
    p_spl0601->i32rawTemperature= (p_spl0601->i32rawTemperature&0x800000) ? (0xFF000000|p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 Function: spl0601_get_raw_pressure
 Description: obtain the original pressure value and turn them into 32bits-integer
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    static uint8_t temp[3];
    HAL_I2C_Mem_Read(&hi2c1,HW_ADR<<1,0x00,I2C_MEMADD_SIZE_8BIT,temp,3,100);    
    p_spl0601->i32rawPressure = ((int32_t)temp[0]<<16) |( (int32_t)temp[1]<<8) | ((int32_t)temp[2]);
    p_spl0601->i32rawPressure= (p_spl0601->i32rawPressure&0x800000) ? (0xFF000000|p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}

/*****************************************************************************
 Function: spl0601_get_temperature
 Description:  return calibrated temperature value base on original value.
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate =  p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 Function: spl0601_get_pressure
 Description: return calibrated pressure value base on original value.
 Input: void            
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc* p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}




