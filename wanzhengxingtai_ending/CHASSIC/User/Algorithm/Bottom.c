#include "Bottom.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞 COPY FORM 澍哥
 *	@performance:	    //带偏移量的
 *	@parameter:		    //
 *	@time:				//22-11-25 14:08
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t MecanumInit(mecanumInit_typdef *mecanumInitT)
{
    /*初始化参数*/
    mecanumInitT->deceleration_ratio = 0.052075f; // 减速比1/19
    mecanumInitT->max_vw_speed       = 50000;     // r方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vx_speed       = 50000;     // x方向上的最大速度单位：毫米/秒
    mecanumInitT->max_vy_speed       = 50000;     // y方向上的最大速度单位：毫米/秒
    mecanumInitT->max_wheel_ramp     = 8000;      // 3508最大转速不包含限速箱
    mecanumInitT->rotate_x_offset    = 00.0f;     // 云台在x轴的偏移量  mm
    mecanumInitT->rotate_y_offset    = 00.0f;     // 云台在y轴的偏移量  mm
    mecanumInitT->wheelbase          = 300;       // 轮距	mm
    mecanumInitT->wheeltrack         = 300;       // 轴距	mm
    mecanumInitT->wheel_perimeter    = 478;       // 轮子的周长(mm)

    /*计算旋转比率*/
    mecanumInitT->raid_fr = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_fl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f -
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_bl = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset - mecanumInitT->rotate_y_offset) /
                            57.3f;
    mecanumInitT->raid_br = ((mecanumInitT->wheelbase + mecanumInitT->wheeltrack) / 2.0f +
                             mecanumInitT->rotate_x_offset + mecanumInitT->rotate_y_offset) /
                            57.3f;
// 将算出来的数据转化到转每分钟上去 raid = 60/(电机减速比*轮的周长)
    mecanumInitT->wheel_rpm_ratio = 60.0f / (mecanumInitT->wheel_perimeter * mecanumInitT->deceleration_ratio);

    return 0;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞 CPOY  FORM 澍哥
*	@performance:	    //带中心点的麦轮解算。可以让车辆绕着一个中心点转动,在移动时保证以操作手为正方向
*	@parameter:		    //@平移速度@前进速度@旋转速度
*	@time:				//22-11-25 13:44
*	@ReadMe:			//左负右正//前正后负//顺正逆负
                        //带有中心偏移量，可以调节旋转中心
                        //要在初始化函数结束之后才能调用
                             矢量表     	//箭头代表每一个麦轮的分力方向，数字带表ID
                              车头
                            ↑  ||  ↑
                            2→ || ←1
                            —————————   Y轴  左正右负，上正下负
                            3→ || ←4
                            ↓  ||  ↓
                              车尾
                        //目前仅仅支持到速度环
************************************************************万能分隔符**************************************************************/

void MecanumResolve(float *wheel_rpm, float vx_temp, float vy_temp, float vr, mecanumInit_typdef *mecanumInit_t)
{
    wheel_rpm[0] = (-vx_temp + vy_temp + vr * mecanumInit_t->raid_fr) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[1] = (vx_temp + vy_temp + vr * mecanumInit_t->raid_fl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[2] = (vx_temp - vy_temp + vr * mecanumInit_t->raid_bl) * mecanumInit_t->wheel_rpm_ratio;
    wheel_rpm[3] = (-vx_temp - vy_temp + vr * mecanumInit_t->raid_br) * mecanumInit_t->wheel_rpm_ratio;
    // // 寻找四个轮子里面的最大值
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     if (RUI_F_MATH_ABS_float(wheel_rpm[i]) > max)
    //     {
    //         max = RUI_F_MATH_ABS_float(wheel_rpm[i]);
    //     }

    // }
    // // 如果超出了最大值，则进行等比例降速
    // if (max > mecanumInit_t.max_wheel_ramp)
    // {
    //     float rate = mecanumInit_t.max_wheel_ramp / max;
    //     for (uint8_t i = 0; i < 4; i++)
    //         wheel_rpm[i] *= rate;
    // }
}

uint8_t OmniInit(OmniInit_typdef *OmniInitT)
{
    /*初始化参数(目前都是瞎给的)*/
    OmniInitT->wheel_perimeter = 478;
    OmniInitT->CHASSIS_DECELE_RATIO = 0.052075f;
    OmniInitT->LENGTH_A = 300;
    OmniInitT->LENGTH_B = 300;
    return 0;
}

void Omni_calc(float *wheel_rpm, float vx_temp, float vy_temp, float vr, OmniInit_typdef *OmniInit_t)
{
    float wheel_rpm_ratio;
    wheel_rpm_ratio = 60.0f/(OmniInit_t->wheel_perimeter*3.14f)*OmniInit_t->CHASSIS_DECELE_RATIO*1000;

    wheel_rpm[0] = (   vx_temp + vy_temp + vr * (OmniInit_t->LENGTH_A+OmniInit_t->LENGTH_B))*wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = (   vx_temp - vy_temp + vr * (OmniInit_t->LENGTH_A+OmniInit_t->LENGTH_B))*wheel_rpm_ratio;//forward
    wheel_rpm[2] = (  -vx_temp - vy_temp + vr * (OmniInit_t->LENGTH_A+OmniInit_t->LENGTH_B))*wheel_rpm_ratio;//right
    wheel_rpm[3] = (  -vx_temp + vy_temp + vr * (OmniInit_t->LENGTH_A+OmniInit_t->LENGTH_B))*wheel_rpm_ratio;//back
}
