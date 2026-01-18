#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "IMU_Task.h"
#include "All_Init.h"
#include "WHW_IRQN.h"

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
		uint16_t romoteOnLine	:1;	//遥控是否在线
		uint16_t chMod :2;			//底盘状态
		uint16_t topSate:1;			//陀螺仪状态
		uint16_t target:1;
		uint16_t DBUS_state:1;
	}dataNeaten;
	//CAN发送的数据
	uint8_t sendData[8];
//	uint8_t getData[8];
};

union IMU_Data_t
{
	struct __packed
	{
		float pitch;
		float yaw;
	}angle;
	uint8_t Data[8];
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
		uint64_t maxSpeed:8;		//最大射速
		uint64_t nowSpeed:8;		//当前射速
		uint64_t target:1;			//是否识别成功标志位
		uint64_t visionMod:3;		//视觉的状态
		uint64_t visionState:1;		//视觉在线的状态
		uint64_t judgeState:1;		//裁判系统的状态
		uint64_t :0;				//保留
	}dataNeaten_another;
	//CAN发送的数据
//	uint16_t sendData[4];
	uint8_t getData[8];
};
struct CanCommunit_typedef
{
		union chTOgm_typdef chTOgm;
		union gmTOch_typdef gmTOch;
};
uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    RUI_ROOT_STATUS_Typedef *Root,
                    MOTOR_Typdef *MOTOR,
                    IMU_Data_t *IMU);
struct GimbalCanRX_typedef 
{
		float pitchData;
		float yawData;
		uint32_t tiem;
		uint8_t bullt;
		uint8_t ammoNumber;
		uint8_t visionState;
		uint8_t shootBoll;
		uint8_t attackMod;
		uint8_t gimbalMod;
		uint8_t target; 
}__attribute__ ((__packed__));
struct KeyboardResolve_typedef
{
		int16_t vx;
		int16_t vy;
		int16_t vr;
}__attribute__ ((__packed__));
//云台结构体
struct gimbal_typedef
{
		struct GimbalCanRX_typedef CanResevie;
		struct KeyboardResolve_typedef Keyboard;
};
uint8_t GimbalRXResolve(uint8_t * buff,uint16_t CANID) ;
uint8_t CANGimbalTX(DBUS_Typedef *WHW_V_DBUS);
void Quaternion_testing(IMU_Data_t *IMU);
float pitch_caculate(IMU_Data_t *IMU);
void Encodeing_control(MOTOR_Typdef *MOTOR,DBUS_Typedef *WHW_V_DBUS);//编码器控制云台电机
uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typdef *MOTOR);

#endif
