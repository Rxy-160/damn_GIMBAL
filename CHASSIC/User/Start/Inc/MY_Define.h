/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-04 16:39:33
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:48:31
 */
#ifndef __MY_DEFINE
#define __MY_DEFINE

//INIT成功
#define RUI_DF_READY 1
//INIT错误
#define RUI_DF_ERROR 0

#define RUI_DF_NOW  1
#define RUI_DF_LAST 0

//PID双环
#define RUI_DF_PID_DOUBLE 1
//PID单环
#define RUI_DF_PID_SINGLE 0
//正在判断主控位置
#define RUI_DF_MASTER_LOCATION_JUDGEING 0
//主控位置判断完成，在头部
#define RUI_DF_MASTER_LOCATION_HEAD 1
//主控位置判断完成，在底盘
#define RUI_DF_MASTER_LOCATION_BOTTOM 2

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	//LED
 *	@parameter:		//
 *	@time:				//23-04-13 15:57
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
#define RUI_DF_LED_RED    0
#define RUI_DF_LED_GREEN  1
#define RUI_DF_LED_BLUE   2
#define RUI_DF_LED_YELLOW 3
#define RUI_DF_LED_PURPLE 4
#define RUI_DF_LED_CYAN   5
#define RUI_DF_LED_WRITE  6
//这个角度设完之和实际的正方向是跟他正好相反的（转180度）
#define INIT_ANGLE 3503

#define WIPE_MAX_SPEED 7200
#define SINGLE_ANGLE -36864
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //离线检测宏定义
 *	@parameter:		    //
 *	@time:				//23-03-28 20:13
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
//设备离线
#define RUI_DF_OFFLINE 0
//设备在线
#define RUI_DF_ONLINE  1 
//遥控器离线检测时间
#define RUI_DF_DBUS_OFFLINE_TIME 30  
//电机离线检测时间
#define RUI_DF_MOTOR_OFFLINE_TIME 15
//电容离线检测时间
#define RUI_DF_CAP_OFFLINE_TIME 15

#define TEMPERATURE_PID {   1600.0f,    0.02f,  5.0f    }
#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID
#define TEMPERATURE_PID_MAX_IOUT 600.0f //max iout of temperature control PID

/////////////////新视觉
// CAN ID
#define CAN_D_1 0
#define CAN_D_2 1
// 底盘
#define CAN_D_CHASSIS_1 0x201
#define CAN_D_CHASSIS_2 0x202
#define CAN_D_CHASSIS_3 0x203
#define CAN_D_CHASSIS_4 0x204
// 电容
#define CAN_D_CAP 0x206
// 陀螺仪
#define CAN_D_TOP 0x101
// 云台
#define CAN_D_GIMBAL_YAW 0x206
#define CAN_D_GIMBAL_PIT 0x208
// 发射
#define CAN_D_ATTACK_L 0x201
#define CAN_D_ATTACK_R 0x202
#define CAN_D_ATTACK_G 0x203

// 底盘电机
#define MOTOR_D_CHASSIS_1 0
#define MOTOR_D_CHASSIS_2 1
#define MOTOR_D_CHASSIS_3 2
#define MOTOR_D_CHASSIS_4 3
// 云台电机
#define MOTOR_D_GIMBAL_YAW 0
#define MOTOR_D_GIMBAL_PIT 1
#define MOTOR_D_ATTACK_L 0
#define MOTOR_D_ATTACK_R 1
#define MOTOR_D_ATTACK_G 2
// 电机类型
#define MOTOR_TYPE_3508 1
#define MOTOR_TYPE_3510 2
#define MOTOR_TYPE_6020 3
#define MOTOR_TYPE_2006 4

// PID所需区分电机宏定义
#define PID_D_CHASSIS 0
#define PID_D_GIMBAL 1
#define PID_D_ATTACK 2
#define PID_D_SPEED 0
#define PID_D_ANGLE 1
#define PID_D_CURRENT 2
// 离线监测
#define NOW 1
#define LAST 0

#define MONITOR_OFFLINE false
#define MONITOR_ONLINE true

// 模式
#define DBUS_D_MOD_SINGLE 2
#define DBUS_D_MOD_CONSIST 1
#define DBUS_D_MOD_SHUT 3

// ROOT信号，涉及程序运行状态
#define ROOT_READY 1
#define ROOT_ERROR 0


// 编译命令，用于调试
#define OLDHEAD 1
// #define OMNI 0
#define LIFTED_DEBUG 0

// 云台限度
#define GIMBAL_PIT_MAX 35 //-100
#define GIMBAL_PIT_MIN -17  //300
#define OMNI_PIT_MAX 5200
#define OMNI_PIT_MIN 4270



//双板通讯
#define CHASSIC_kong 0x232
#define GIMBAL_kong 0x231

#endif
