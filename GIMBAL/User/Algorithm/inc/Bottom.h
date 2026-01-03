#ifndef BOTTOM_H
#define BOTTOM_H

#include "main.h"
#include "MY_Define.h"

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float wheeltrack;         /* 轮距（mm）*/
    float wheelbase;          /* 轴距（mm）*/
    float rotate_x_offset;    /* 相对于底盘中心的x轴旋转偏移量(mm) */
    float rotate_y_offset;    /* 相对于底盘中心的y轴旋转偏移量(mm) */
    float deceleration_ratio; /* 电机减速比 */
    int max_vx_speed;         /* 底盘的x轴的最大速度(mm/s) */
    int max_vy_speed;         /* 底盘的y轴的最大速度(mm/s) */
    int max_vw_speed;         /* 底盘的自转的最大速度(degree/s) */
    int max_wheel_ramp;       /* 3508最大转速 */
    // 每一个轮子的旋转比率//与旋转中心点相关
    float raid_fr;            // 右前
    float raid_fl;            // 左前
    float raid_bl;            // 左后
    float raid_br;            // 右后
    float wheel_rpm_ratio;    // 用来将速度转化成转每分钟
} mecanumInit_typdef;

typedef struct
{
    float wheel_perimeter;    /* 轮的周长（mm）*/
    float CHASSIS_DECELE_RATIO; /* 电机减速比 */
    float LENGTH_A;         /* 底盘长的一半（mm）*/
    float LENGTH_B;          /* 底盘宽的一半（mm）*/
} OmniInit_typdef;

uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT);
void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t);

uint8_t OmniInit(OmniInit_typdef *OmniInitT);
void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t);

#endif
