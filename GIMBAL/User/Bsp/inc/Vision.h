#ifndef _VISION_H_
#define _VISION_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "MY_Define.h"
#include "RUI_MATH.h"
#include "RUI_DBUS.h"
#include "Referee.h"
#include "IMU_Task.h"
union RUI_U_VISION_RECEIVE
{
  struct
  {
    // uint8_t HEAD;
    float PIT_DATA;
    float YAW_DATA;
    bool TARGET;
    bool fire;
    bool state;
  };
  uint8_t DATA[15];
};
// #pragma pack(pop)

union RUI_U_VISION_SEND
{
  struct
  {
    float PIT_DATA;
    float YAW_DATA;
		float ROLL_DATA;
    float INIT_FIRING_RATE; // 弹速
    int FLAG;            // 自瞄和能量机关切换标志位
    bool COLOR;             // TRUE是蓝色，FALSE是红色
    uint32_t TIME;
    uint8_t bulletSpeed;//
    uint8_t is_buff; // 0:自瞄 1:打符
//		uint8_t 
  };
  uint8_t DATA[21];
};

typedef struct TYPEDEF_VISION
{
  union RUI_U_VISION_RECEIVE RECEIVE;
  union RUI_U_VISION_SEND SEND;
  uint8_t OriginData[20];
  uint8_t RECV_FLAG[2];
  int RECV_OutTime;
  uint32_t block_Time;
}TYPEDEF_VISION;

typedef union ReceiveDataUnion_typedef							//共用体(用于接受各种数据)(视觉，陀螺仪)
	{    
			uint8_t U[4];
			float F;
			uint32_t I;
	}VisionTemp;



uint8_t VISION_F_Cal(uint8_t *RxData, uint8_t type,TYPEDEF_VISION *VISION_DATA);
int ControltoVision(union RUI_U_VISION_SEND*  Send_t , uint8_t *buff, uint8_t type,User_Data_T *users,DBUS_Typedef*DBUS,IMU_Data_t *IMU_Data,TYPEDEF_VISION *VISION_V_DATA);
void VisionSendInit(union RUI_U_VISION_SEND*  Send_t,TYPEDEF_VISION *VISION_DATA,User_Data_T*User_Data_aaa,DBUS_Typedef *DBUS_sss,IMU_Data_t *IMU_Data);
void VISION_F_Monitor(TYPEDEF_VISION *VISION_V_DATA);


	
	
	
	///////Top.h
	
	
	typedef struct TYPEDEF_TOP_DATA
{
    float REALITY_ANGLE; // 真实角度//带圈数
    int16_t YAW_ANGLE[2];
    float YAW_ANGLE_F;
    int16_t YAW_SPEED[2];
    int16_t PIT_ANGLE[2];
    float PIT_ANGLE_F;
    int16_t PIT_SPEED[2];
    int16_t ROUND;
} TYPEDEF_TOP_DATA;

typedef union TYPEDEF_TOP_DATA_UNION // 共用体(用于接受各种数据)(视觉，陀螺仪)
{
    struct
    {
        int16_t YAW_ANGLE;
        int16_t PIT_ANGLE;
        int16_t YAW_SPEED;
        int16_t PIT_SPEED;
    };
    uint8_t GET_DATA[8];
} TYPEDEF_TOP_DATA_UNION;


typedef struct TYPEDEF_TOP
{
    float yaw[6];   // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
    float pitch[6]; // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
    float roll[6];  // add by yu 1-now 0-last 2-laps 3-infinite 4-0-close/1-open
}TYPEDEF_TOP;

extern float yaw, pitch, roll;
//extern QEKF_INS_t QEKF_INS;

// extern float Top[5];
extern TYPEDEF_TOP TOP;
extern float currentAngle; // 陀螺仪yaw偏差角度
extern void TOP_T_Cal();
extern void TOP_T_Monitor();

void TOP_T_Cal_T();

#endif
