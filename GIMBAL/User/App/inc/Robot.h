#ifndef __ROBOT_H
#define __ROBOT_H

#include "RUI_DBUS.h"
#include "Motors.h"
#include "Power_CAP.h"
#include "Vision.h"
#include "IMU_Task.h"
#include "controller.h"




void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               TYPEDEF_VISION *Vision/*普通视觉*/
								/*VisionRxDataUnion *Vision 加预测视觉*/,
               RUI_ROOT_STATUS_Typedef *Root,
               MOTOR_Typdef *MOTOR,
               IMU_Data_t *IMU_Data,
							 TD_t *TDDD);

float RUI_F_GET_FIRE_WIPE_SPEED(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS,
                                User_Data_T *User_data, RUI_ROOT_STATUS_Typedef *Root);

static int64_t RUI_F_GET_FIRE_AIM(DBUS_Typedef *DBUS,
                                  CONTAL_Typedef *CONTAL,
                                  User_Data_T *User_data);

float RUI_F_CHASSIS_GET_MAX_TARGET(float MAX_POWER);

float RUI_F_CHASSIS_PID(int16_t RELATIVE_ANGLE, float KP, float KI, float KD);

#endif
