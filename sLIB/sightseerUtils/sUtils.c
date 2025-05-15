#include "sUtils.h"




//把一个浮点数从一个范围转换到另一个范围
float inline sut_fmap(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//把一个整数从一个范围转换到另一个范围
uint32_t inline sut_imap(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//快速求平方根倒数
float sut_isqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//从气压转换到高度,返回单位m
float sut_press2alt(float press_pa){
    //匿名科创的算法,返回m
    float x = (101000.0f - press_pa) / 1000.0f;
    return 0.82f * x * x * x + 0.9f * x * 100.0f;
}

//比较两个浮点数的大小,可以用于qsort的比较函数,返回1: a>b, 0: a==b(可能不精确), -1: a<b
int sut_fcmp(const void *a, const void *b){
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

//判断两个浮点数是否相等,返回1: 相等, 0: 不相等
int inline sut_fequal(float fa, float fb){
    //如果差值小于FLT_EPSILON,则认为两数相等
    if(fabs(fa - fb) < FLT_EPSILON){
        return 1;  //相等
    }else{
        return 0;  //不相等
    }
}

//获得航向角的差值
float inline sut_headingdiff(float heading1, float heading2){
    float diff = heading1 - heading2;
    while (diff < -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;
    return diff;
}

//浮点数限幅
void inline sut_fconstrain(float *val, float min, float max){
    if(*val > max) *val = max;
    if(*val < min) *val = min;
}


//伽马校正,把输入的亮度百分比0~100% -> 0~100PWM
float sut_gammacorr(float percent_brightness){
    #define GAMMA_VAL 2.2f
    //将输入的亮度百分比转换为0到1之间的范围
    float normalized_brightness = percent_brightness / 100.0;
    //应用伽马校正公式
    float corrected_brightness = powf(normalized_brightness, GAMMA_VAL);
    //将校正后的亮度映射回PWM占空比范围
    float pwm_duty_cycle = corrected_brightness * 100.0;
    return pwm_duty_cycle;
}

/**
 * @brief 修改寄存器中的特定位段，只在需要时更新。
 * 这个函数首先计算掩码中最低位的位置，然后将输入数据左移对齐到这个位置。
 * 之后，它会计算需要更新的位，并只更新这些位，其他位保持不变。
 *
 * @param reg_addr 寄存器的地址，通常是一个指向uint8_t的指针。
 * @param mask 指定要修改的位，例如0b00111000表示修改第4到第6位。
 * @param data 要写入寄存器位段的数据，数据需要是从最低位开始并只包含目标位段。
 */
void sut_ModifyReg(uint8_t* reg_addr, uint8_t mask, uint8_t data){
    uint8_t pos = 0;
    uint8_t mask_original = mask;
    // 计算掩码最低位的位置
    while ((mask & 0x01) == 0) {
        mask >>= 1;
        pos++;
    }
    // 将数据左移，对齐到掩码指定的位
    uint8_t aligned_data = (data << pos) & mask_original;
    // 读取当前寄存器值
    uint8_t current_value = *reg_addr;
    // 计算需要变更的位
    uint8_t changes = (current_value & mask_original) ^ aligned_data;
    // 只更新变化的位，不影响其他位
    uint8_t new_value = (current_value & ~changes) | (aligned_data & changes);
    // 写回修改后的值到寄存器
    *reg_addr = new_value;
}

//初始化SWF,传入滤波器句柄,窗口大小
void sut_swf_init(sut_swf_t* pFliter,uint8_t win_size){
    pFliter->size = win_size;
    pFliter->data = (float*)sut_malloc(win_size * sizeof(float));
    memset(pFliter->data, 0, win_size * sizeof(float));
    pFliter->index = 0;
    pFliter->sum = 0.0;
    pFliter->average = 0.0;
}

//释放SWF
void sut_swf_deinit(sut_swf_t* pFliter){
    sut_free(pFliter->data);
    pFliter->data = NULL;
    pFliter->size = 0;
    pFliter->index = 0;
    pFliter->sum = 0.0;
    pFliter->average = 0.0;
    pFliter->is_frist = true;
}

//传入滤波器和新数据,返回平均值
float sut_swf_update(sut_swf_t* pFliter,float newData){
    if(pFliter->is_frist){
        //第一次更新,把窗口里填满相同数据,跳过他从0到目标值的收敛过程
        for(uint8_t i = 0; i < pFliter->size; i++){
            pFliter->data[i] = newData;
        }
        pFliter->is_frist = false;
    }
    //减去即将被覆盖的数据点的值
    pFliter->sum -= pFliter->data[pFliter->index];
    //添加新的数据到窗口并更新索引
    pFliter->data[pFliter->index] = newData;
    pFliter->sum += newData;
    pFliter->index = (pFliter->index + 1) % pFliter->size;
    //计算新的平均值
    pFliter->average = pFliter->sum / pFliter->size;
    return pFliter->average;
}

//中值滤波,传入滤波器句柄和窗口大小
void sut_mwf_init(sut_mwf_t* pFilter, uint8_t size){
    pFilter->size = size;
    pFilter->data = (float*)sut_malloc(size * sizeof(float));
    memset(pFilter->data, 0, size * sizeof(float));
    pFilter->index = 0;
}

//传入滤波器和新数据, 返回中值
float sut_mwf_update(sut_mwf_t* pFilter, float newData){
    //更新数据
    pFilter->data[pFilter->index] = newData;
    pFilter->index = (pFilter->index + 1) % pFilter->size;
    
    //创建一个用于排序的临时数组
    float sortedData[pFilter->size];
    memcpy(sortedData, pFilter->data, pFilter->size * sizeof(float));
    qsort(sortedData, pFilter->size, sizeof(float), sut_fcmp);
    
    //返回中值
    if (pFilter->size % 2 == 1) {
        return sortedData[pFilter->size / 2];
    } else {
        //如果数据数量是偶数，则取中间两个数的平均值
        return (sortedData[pFilter->size / 2 - 1] + sortedData[pFilter->size / 2]) / 2.0;
    }
}




