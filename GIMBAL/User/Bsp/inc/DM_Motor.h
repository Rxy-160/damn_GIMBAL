#ifndef __DM_MOTOR_H
#define __DM_MOTOR_H

#include "RUI_MATH.h"
#include "controller.h"
#include "MY_define.h"
#include "can_bsp.h"

#define MIT_MODE  0x000
#define POS_MODE  0x100
#define SPEED_MODE  0x200

#define P_MIN   -12.5f
#define P_MAX   12.5f
#define V_MIN   -30.0f
#define V_MAX   30.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -10.0f
#define T_MAX   10.0f
extern int16_t cur_int16;

typedef struct
{
		int id;
	int state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;

    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
	  int16_t acceleration;//加速度
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;

    int8_t ONLINE_JUDGE_TIME;
	uint16_t initialAngle;
		float ralativeAngle;
	int16_t round;
		int32_t reality;////连续编码器


}DM_MOTOR_DATA_Typdef;

typedef struct
{
    uint8_t PID_INIT;
    DM_MOTOR_DATA_Typdef DATA;
    Feedforward_t PID_F;
    PID_t PID_P;
    PID_t PID_S;
}DM_MOTOR_Typdef;

typedef enum
{
  DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
  DM_CMD_RESET_MODE = 0xfd,   // 停止
  DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
  DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;


typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
	float aim;
	float cur;
	int16_t pos[2];
  int16_t vel[2];
	int16_t round;
	int32_t reality;
	uint16_t initialAngle;
	float ralativeAngle;

}motor_fbpara_t;
typedef struct 
{
	//初始化需要将该项化零
	float kp;
	float ki;
	float kd;
	float epsilon;
	int32_t iLt;
	int32_t alLt;
	float error[2];
	float p_out;
	float i_out;
	int32_t d_out;
	float all_out;
}PID_typedef;
typedef struct {
	float A;
	float B;
    float rin;
    float lastRin;
    float perrRin;
}FFC_typedef;

typedef struct
{
	uint16_t mode[2];
	motor_fbpara_t para;
  FFC_typedef FFC;
  PID_typedef PID_P;
  PID_typedef PID_S;
}Joint_Motor_t ;


float uint_to_float(int x_int, float x_min, float x_max, int bits);
void dm4310_fbdata(DM_MOTOR_DATA_Typdef *motor, uint8_t *rx_data);
void motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what);
void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float vel);
void dm4310_current_set(hcan_t* hcan, uint16_t id, float m1_cur_set, float m2_cur_set, float m3_cur_set, float m4_cur_set);
void dm4310_RXdata(DM_MOTOR_Typdef *motor, uint8_t *rx_data); //一拖四模式下
int16_t OneFilter1(int16_t last, int16_t now, float thresholdValue);
void dm_motor_init(void);
void pos_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);

#endif
