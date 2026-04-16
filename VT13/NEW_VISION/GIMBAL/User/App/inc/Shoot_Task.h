#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "RUI_DBUS.h"
#include "iir.h"


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
void ATTACK_F_JAM_Aim(MOTOR_Typdef *MOTOR, VT13_Typedef *VT13_DBUS, uint8_t autofire);
void ATTACK_F_FIRE_Aim(MOTOR_Typdef *MOTOR,VT13_Typedef *VT13_DBUS);
void ATTACK_F_Ctl(VT13_Typedef *VT13_DBUS,MOTOR_Typdef *MOTOR);
uint8_t MOTOR_PID_Shoot_INIT(MOTOR_Typdef *MOTOR);

//float ATTACK_F_FireRate_Control(TYPEDEF_MOTOR *motor, float hz, uint8_t type);
uint8_t ATTACK_F_HeatControl(MOTOR_Typdef *MOTOR, uint8_t type,User_Data_T *User_Data) ;
void static_word(VT13_Typedef *VT13_DBUS);
void shoot_status(MOTOR_Typdef *MOTOR);
extern uint8_t shoot_stateee;////
extern uint8_t motor_F_state;
extern uint8_t motor_M_state;

extern uint16_t statues;//检测弹丸累计

extern double filler_motor_L;

extern double filler_motor_R;
extern double filler_motor_M;

typedef struct {
    float alpha;        // 滤波系数 (0-1)，越小越平滑，越大响应越快
    float last_output;  // 上次输出值
    int initialized;    // 是否已初始化
} LowPassFilter;

extern LowPassFilter LowPass;
void lpf_init(LowPassFilter *f, float alpha) ;
float lpf_process(LowPassFilter *f, float input) ;



///////////以下是火控的弹丸检测部分

//这下面是射击检测的
#define K_UP             0.673//0.360f   // 上升系数
#define K_DN             0.142//0.059f   // 下降系数
#define TH_FIRE          200.0f   // 触发阈值
#define TH_FIRE_MAX      1200.0f  // 最大触发阈值
#define MIN_SLOPE        80.0f    // 最小斜率阈值
#define RELATIVE_RECOVER 0.25f    // 回升比例
#define TH_RST_SAFE      100.0f   // 复位阈值
#define TIMEOUT_TICKS    14//35       // 超时上限
#define COOL_DOWN_TICKS  2//5        // 冷却周期

// 重新定义的结构体
typedef struct {
    float base;              // 动态基准线
    float last_val;          // 记录上一次的转速，用于算斜率
    float max_drop_in_round; // 记录单次触发过程中的最大跌落深度
    bool  armed;             // 触发状态
    uint32_t cnt;            // 计数器
		uint32_t last_cnt;     //上一次的计数器
    uint8_t  t_out;          // 超时计数器
    uint8_t  cool_down_cnt;  // 冷却计数器
    bool  init;              // 初始化标志
} ShootDet_t;


bool Update_Shoot_Det(float speed1, float speed2, ShootDet_t *det);

extern bool text_shoot_cnt;
extern ShootDet_t g_det;



#endif
