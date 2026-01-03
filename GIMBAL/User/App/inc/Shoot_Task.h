#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "RUI_DBUS.h"
#define DBUS_D_MOD_SINGLE 2    //单发
#define DBUS_D_MOD_CONSIST 1   //连发
#define DBUS_D_MOD_SHUT 3      //拨盘在中间

uint8_t shoot_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   MOTOR_Typdef *MOTOR);


typedef struct 
{
    int32_t TIME;
    int8_t FLAG;        // 卡弹变换方向
	  float SINGLE_ANGLE; // 单发角度
    float SPEED;        // 摩擦轮速度
    int COUNT;
    int8_t LOCK;
    uint8_t STATUS[2];      // 拨盘到达 status 0到达 1未到
    uint8_t PREV_MOUSE_STATE; // 上次左鼠标状态，控制拨盘旋转
    uint8_t fire_wheel_status; // 摩擦轮状态
    float jam_dwt_time;
    uint8_t is_jam;      // 卡弹状态
}TYPEDEF_ATTACK_PARAM;

extern TYPEDEF_ATTACK_PARAM ATTACK_V_PARAM;

void ATTACK_F_Init(MOTOR_Typdef *MOTOR);
void ATTACK_F_JAM_Aim(MOTOR_Typdef *MOTOR, DBUS_Typedef *DBUS, uint8_t autofire);
void ATTACK_F_FIRE_Aim(MOTOR_Typdef *MOTOR,DBUS_Typedef*DBUS);
void ATTACK_F_Ctl(DBUS_Typedef *DBUS,MOTOR_Typdef *MOTOR);
uint8_t MOTOR_PID_Shoot_INIT(MOTOR_Typdef *MOTOR);


#endif
