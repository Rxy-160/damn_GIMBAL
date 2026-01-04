/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2023-12-31 13:49:55
 * @LastEditors: Andy
 * @LastEditTime: 2024-05-04 14:46:31
 */

#ifndef RUI_ROOT_INIT_H
#define RUI_ROOT_INIT_H
#include "main.h"
#include "stdint.h"
#include "RUI_MATH.h"
#include "MY_define.h"
#include "RUI_DBUS.h"
#include "usart.h"
#include "bsp_dwt.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "Gimbal_Task.h"
#include "RGBctrl.h"
#include "Motors.h"
#include "Power_CAP.h"

//在定时器调用这一个函数即可
void RUI_F_ROOT(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET);
//总的状态监测
void RUI_F_ROOT_ALL_STATUS(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET);//主控位置判断
uint8_t RUI_F_MASTER_LOCATION_JUDGE(RUI_ROOT_STATUS_Typedef *Root);
//主控通信
//uint8_t RUI_F_MASTER_COMMUNICATION(void);
//LED
void RUI_F_LED_SHOW_STATUS(RUI_ROOT_STATUS_Typedef *Root);
#endif

