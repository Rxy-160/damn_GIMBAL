#ifndef _SMC_H_
#define _SMC_H_

#include <stdint.h>
#include <math.h>
#include "stm32f4xx_hal.h"

typedef struct {
    // 参数
    float C;
    float K;
    float ref;
    float error_eps;
    float u_max;
    float J;
    float epsilon;
    
    // 状态变量
    float angle;
    float ang_vel;
    float u;
    
    // 私有变量
    float error;
    float error_last;
    float dref;
    float ddref;
    float refl;
    float s;
} SMC;

// 初始化函数
void SMC_Init(SMC* smc, float C, float K, float ref, float error_eps, float u_max, float J, float epsilon);

// 控制更新函数
void SMC_Tick(SMC* smc, float angle_now, float angle_vel);

// 全局实例初始化（在main中调用）
void Init_YawSMC(SMC* smc);

// 全局实例计算（在main中调用）注意角度为°，角速度为°/s
float Tick_YawSMC(SMC* smc, float angle_now, float angle_vel, float ref);

#endif
