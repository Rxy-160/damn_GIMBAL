#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "RUI_DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"

uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                     RUI_ROOT_STATUS_Typedef *Root,
                     User_Data_T *User_data,
                     model_t *model,
                     CAP_RXDATA *CAP_GET,
                     MOTOR_Typdef *MOTOR);


#endif
