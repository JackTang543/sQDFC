#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <float.h>

#include "defines.h"
#ifdef RELEASE_CODE_MODE

#else
#define SUTILS_OUTPUT_ENABLE
#endif

#include "FreeRTOS.h"

// 使用哪种malloc和free
#define sut_malloc pvPortMalloc
#define sut_free   vPortFree

// PI
#define M_PI (3.14159f)
// EXP
#define M_EXP (2.71828f)

// 重力常数
#define M_GRAVITY (9.81398f)

// 度转弧度
#define DEG2RAD (0.017453f)
// 弧度转度
#define RAD2DEG (57.29578f)

// 工程转速转弧度每秒
#define RPM2RADPS (0.104720f)
// 弧度每秒转工程转速
#define RADPS2RPM (9.549297f)

// 把一个浮点数从一个范围转换到另一个范围
float sut_fmap(float x, float in_min, float in_max, float out_min, float out_max);
// 把一个整数从一个范围转换到另一个范围
uint32_t sut_imap(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
// 快速求平方根倒数
float sut_isqrt(float x);
// 从气压转换到高度,返回单位m
float sut_press2alt(float press_pa);
// 比较两个浮点数的大小,可以用于qsort的比较函数,返回1: a>b, 0: a==b, -1: a<b
int sut_fcmp(const void* a, const void* b);
// 比较两个浮点数是否相等
int sut_fequal(float fa, float fb);
// 获得航向角的差值
float sut_headingdiff(float heading1, float heading2);
// 浮点数限幅
void sut_fconstrain(float* val, float min, float max);
// 伽马矫正
float sut_gammacorr(float percent_brightness);

/**
 * @brief 修改寄存器中的特定位段，只在需要时更新。
 * 这个函数首先计算掩码中最低位的位置，然后将输入数据左移对齐到这个位置。
 * 之后，它会计算需要更新的位，并只更新这些位，其他位保持不变。
 *
 * @param reg_addr 寄存器的地址，通常是一个指向uint8_t的指针。
 * @param mask 指定要修改的位，例如0b00111000表示修改第4到第6位。
 * @param data 要写入寄存器位段的数据，数据需要是从最低位开始并只包含目标位段。
 */
void sut_modify_reg(uint8_t* reg_addr, uint8_t mask, uint8_t data);

// SWF滑窗滤波
typedef struct {
    uint8_t size;  // 窗口大小
    float* data;   // 数据缓冲区
    uint8_t index; // 当前数据索引
    float sum;     // 数据的总和，用于快速计算平均值
    float average; // 当前的平均值
    bool is_frist; // 是否第一次更新
} sut_swf_t;

void sut_swf_init(sut_swf_t* pFliter, uint8_t win_size);
void sut_swf_deinit(sut_swf_t* pFliter);
float sut_swf_update(sut_swf_t* pFliter, float newData);

// MWF中值滤波
typedef struct {
    uint8_t size;  // 窗口大小
    float* data;   // 数据缓冲区
    uint8_t index; // 当前数据索引
} sut_mwf_t;

void sut_mwf_init(sut_mwf_t* pFilter, uint8_t size);
float sut_mwf_update(sut_mwf_t* pFilter, float newData);

typedef struct {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float setpoint;     // 设定目标值
    float integral;     // 积分累计
    float prev_error;   // 上一次的误差
    float output_min;   // 输出限制最小值
    float output_max;   // 输出限制最大值
    float integral_max; // 积分限幅最大值
    float integral_min; // 积分限幅最小值
} sut_pos_pid_t;

typedef struct {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float setpoint;     // 设定目标值
    float integral;     // 积分累计
    float prev_error;   // 上一次的误差
    float delta_output; // 输出的变化量
    float output;       // 当前输出值
    float output_min;   // 输出限制最小值
    float output_max;   // 输出限制最大值
    float integral_max; // 积分限幅最大值
    float integral_min; // 积分限幅最小值
} sut_inc_pid_t;

/*适用于SISO系统的一般式PID控制器*/
/*位置式PID*/
void sut_pos_pid_init(sut_pos_pid_t* pid, float out_min, float out_max, float int_min, float int_max);
void sut_pos_pid_setK(sut_pos_pid_t* pid, float Kp, float Ki, float Kd);
void sut_pos_pid_setTar(sut_pos_pid_t* pid, float setpoint);
float sut_pos_pid_getTar(sut_pos_pid_t* pid);
float sut_pos_pid_update(sut_pos_pid_t* pid, float measured_value, float dt_s);
/*增量式PID*/
void sut_inc_pid_init(sut_inc_pid_t* pid, float out_min, float out_max, float int_min, float int_max);
void sut_inc_pid_SetK(sut_inc_pid_t* pid, float Kp, float Ki, float Kd);
void sut_inc_pid_setTar(sut_inc_pid_t* pid, float setpoint);
float sut_inc_pid_getTar(sut_inc_pid_t* pid);
float sut_inc_pid_update(sut_inc_pid_t* pid, float measured_value, float dt_s);

void sUtils_LogAssert(const char* fmt, ...);
void sUtils_LogError(const char* fmt, ...);
void sUtils_LogWarn(const char* fmt, ...);
void sUtils_LogInfo(const char* fmt, ...);
void sUtils_LogPrintf(const char* fmt, ...);
void sUtils_LogPrintfln(const char* fmt, ...);

#ifdef SUTILS_OUTPUT_ENABLE
#define log_assert   sUtils_LogAssert
#define log_error    sUtils_LogError
#define log_warn     sUtils_LogWarn
#define log_info     sUtils_LogInfo
#define log_printf   sUtils_LogPrintf
#define log_printfln sUtils_LogPrintfln
#else
#define log_assert(...)   ((void)0)
#define log_error(...)    ((void)0)
#define log_warn(...)     ((void)0)
#define log_info(...)     ((void)0)
#define log_printf(...)   ((void)0)
#define log_printfln(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif
