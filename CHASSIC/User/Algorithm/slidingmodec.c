#include "slidingmodec.h"
#include <math.h>

// 静态辅助函数声明
static float Sat(float y);
static int8_t Signal(float y);

void SMC_Init(SMC* smc, float C, float K, float ref, float error_eps, 
             float u_max, float J, float epsilon) {
    smc->C = C;
    smc->K = K;
    smc->ref = ref;
    smc->error_eps = error_eps;
    smc->u_max = u_max;
    smc->J = J;
    smc->epsilon = epsilon;
    
    // 初始化状态变量
    smc->refl = 0;
    smc->error = 0;
    smc->dref = 0;
    smc->ddref = 0;
    smc->s = 0;
}

void SMC_Tick(SMC* smc, float angle_now, float angle_vel) {
    smc->angle = angle_now;
    smc->ang_vel = angle_vel;
    smc->error = smc->angle - smc->ref;

    // 计算导数项
    smc->ddref = (smc->ref - smc->refl) - smc->dref;
    smc->dref = (smc->ref - smc->refl);

    // 误差下限处理
    if (fabsf(smc->error) < smc->error_eps) {
        smc->u = 0;
        return;
    }

    // 计算滑模面
    smc->s = smc->C * smc->error + (smc->ang_vel - smc->dref);

    // 计算控制量
    smc->u = smc->J * (smc->ddref - smc->C * (smc->ang_vel - smc->dref) 
                      - smc->epsilon * Sat(smc->s) - smc->K * smc->s);

    // 限幅处理
    if (smc->u > smc->u_max) smc->u = smc->u_max;
    if (smc->u < -smc->u_max) smc->u = -smc->u_max;

    // 更新历史值
    smc->refl = smc->ref;
}

// 饱和函数
static float Sat(float y) {
    return (fabsf(y) <= 1) ? y : (float)Signal(y);
}

// 符号函数
static int8_t Signal(float y) {
    return (y > 0) ? 1 : ((y < 0) ? -1 : 0);
}

// 全局实例初始化（在main中调用）
void Init_YawSMC(SMC* smc) {
    SMC_Init(smc, 20, 120, 0, 0.001f, 25000, 0.8f, 0.5f);
}

// 全局实例计算（在main中调用）注意角度为°，角速度为°/s
float Tick_YawSMC(SMC* smc, float angle_now, float angle_vel, float ref) {
    smc->ref = ref;
    SMC_Tick(smc, angle_now, angle_vel);
    return smc->u;
}
