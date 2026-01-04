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
union gmTOch_typdef		//使用共用体整合数据
{
	struct {			
		int16_t vx:11;		//平移速度
		int16_t vy:11;		//前进速度
		int16_t vr:11;		//旋转速度
		uint16_t key_q:1;
		uint16_t key_e:1;
		uint16_t key_r:1;
		uint16_t key_z:1;
		uint16_t key_x:1;
		uint16_t key_c:1;
		uint16_t key_v:1;
		uint16_t key_shift:1;
		uint16_t key_ctrl:1;
		uint16_t key_f:1;
		uint16_t key_g:1;
		uint16_t key_b:1;
		uint16_t S1:1;
		uint16_t S2:1;
		uint16_t supUSe :1;			//是否使用电容
		uint16_t romoteOnLine	:1;			//遥控是否在线
		uint16_t chMod :2;		//底盘状态
		uint16_t topSate:1;		//陀螺仪状态
//		int16_t ptichAgnle :8;	//陀螺仪发送的pitch轴角度
		uint16_t target:1;
//		uint16_t rala :2;
		uint16_t DBUS_state:1;
	}dataNeaten;
	//CAN发送的数据
//	uint8_t sendData[8];
	uint8_t getData[8];
};
union chTOgm_typdef		//使用共用体整合数据
{
	struct {
		int16_t pitch;
		int16_t yaw;
		float time;
	}dataNeaten_angle;
	struct{
		uint64_t muzzleColing:16;	//枪口热量
		uint64_t maxSpeed:8;	//最大射速
		uint64_t nowSpeed:8;	//当前射速
		uint64_t target:1;	//是否识别成功标志位
		uint64_t visionMod:3;		//视觉的状态
		uint64_t visionState:1;		//视觉在线的状态
		uint64_t judgeState:1;		//裁判系统的状态
		uint64_t :0;		//保留
	}dataNeaten_another;
	//CAN发送的数据
	uint16_t sendData[4];
//	uint8_t getData[8];
};
struct CanCommunit_typedef
{
		union chTOgm_typdef chTOgm;
		union gmTOch_typdef gmTOch;
};
uint8_t ChassisRXResolve(uint8_t * data,DBUS_Typedef *DBUS,RUI_ROOT_STATUS_Typedef *Root);
uint8_t ChassisTXResolve(User_Data_T *User_data);

extern uint16_t ralati;

#endif
