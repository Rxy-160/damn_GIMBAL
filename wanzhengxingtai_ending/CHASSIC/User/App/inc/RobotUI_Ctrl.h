#ifndef __ROBOTUI_CTRL_H
#define __ROBOTUI_CTRL_H

#include "RUI_DBUS.h"
#include "Vision.h"
#include "Referee.h"
#include "Power_CAP.h"
#include "Motors.h"
#include "ui.h"
#include "ui_default_init6_0.h"

uint8_t RobotUI_Static_Init();
void RobotUI_Dynamic(uint8_t RM_DBUS,
                     uint32_t Shoot_Number,
                     float Pitch,
                     float CAP_VOLT,
                     int16_t Angle,
                     int16_t Speed_L_now,
                     int16_t Speed_R_now,
                     TYPEDEF_VISION *Vision);

#endif
