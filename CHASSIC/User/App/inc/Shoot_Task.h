#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "RUI_DBUS.h"

uint8_t shoot_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   MOTOR_Typdef *MOTOR);

#endif
