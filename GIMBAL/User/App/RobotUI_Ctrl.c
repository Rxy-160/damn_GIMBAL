#include "RobotUI_Ctrl.h"

uint8_t RobotUI_Static_Init()
{
    ui_init_default_init0();
    osDelay(34);

    ui_init_default_init1();
    osDelay(34);

    ui_init_default_init2();
    osDelay(34);

    ui_init_default_init3();
    osDelay(34);

    ui_init_default_init4();
    osDelay(34);

    ui_init_default_init5();
    osDelay(34);

    ui_init_default_init6();

    return RUI_DF_READY;
}

void RobotUI_Dynamic(uint8_t RM_DBUS,
                     uint32_t Shoot_Number,
                     float Pitch,
                     float CAP_VOLT,
                     int16_t Angle,
                     int16_t Speed_L_now,
                     int16_t Speed_R_now,
                     TYPEDEF_VISION *Vision)
{
    static uint8_t UI_Init = RUI_DF_READY;

    //遥控离线监测
    if(!RM_DBUS)
    {
        UI_Init = RUI_DF_ERROR;
    }

    //静态UI刷新
    if (UI_Init != RUI_DF_READY)
    {
        UI_Init = RobotUI_Static_Init();
    }

    //动态UI数据修改
    ui_default_init6_Dynamic0->_a++;
    ui_default_init6_Dynamic1->_a++;
    ui_default_init6_Dynamic2->_c++;
    ui_default_init6_Dynamic3->_c++;
    ui_default_init6_Dynamic4->_c++;
    ui_default_init6_Dynamic5->_c++;
    ui_default_init6_Dynamic6->_c++;

    //动态UI刷新
    ui_update_default_init6();
}
