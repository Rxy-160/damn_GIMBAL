#include "usart.h"
#include "Vision.h"

#define VISION_D_SEND 16
//////中南16
#define VISION_D_RECV 15
VisionTemp Union_temp;

#define VISION_D_MONITOR_LEN 10
union ReceiveDataUnion_typedef	data_tackle ={0};
float VisionMonitor[VISION_D_MONITOR_LEN] = {0}; // 只看pit数据是否变化判断离线
int VISION_Monitor_IOTA = 0;
float yaw, pitch, roll;

/// @brief 视觉接收
/// @param RxData 原始数据
/// @param type 类型 0:虚拟串口 1:USART1
/// @return 
uint8_t VISION_F_Cal(uint8_t *RxData, uint8_t type,TYPEDEF_VISION *VISION_DATA)
{
    if ((RxData[0] == 0xcd) && (RxData[VISION_D_RECV - 1] == 0xdc))
    {
        if (type == 0) // 虚拟串口需要从缓冲区中读取数据
            memcpy(VISION_DATA->OriginData, RxData, VISION_D_RECV); 

        // 读取数据
        data_tackle.U[0] = VISION_DATA->OriginData[1];
        data_tackle.U[1] = VISION_DATA->OriginData[2];
        data_tackle.U[2] = VISION_DATA->OriginData[3];
        data_tackle.U[3] = VISION_DATA->OriginData[4];
        VISION_DATA->RECEIVE.PIT_DATA= data_tackle.F;

        data_tackle.U[0] = VISION_DATA->OriginData[5];
        data_tackle.U[1] = VISION_DATA->OriginData[6];
        data_tackle.U[2] = VISION_DATA->OriginData[7];
        data_tackle.U[3] = VISION_DATA->OriginData[8];
         VISION_DATA->RECEIVE.YAW_DATA= data_tackle.F;

        //是否识别到目标    
         VISION_DATA->RECEIVE.TARGET= ( VISION_DATA->OriginData[9] & 0x10)>>4;//识别成功标志位
         VISION_DATA->RECEIVE.fire = ( VISION_DATA->OriginData[9] & 0x08)>>3;
         VISION_DATA->RECEIVE.state = ( VISION_DATA->OriginData[9] & 0x07); 
         VISION_DATA->RECV_FLAG[NOW] = ROOT_READY;

        return ROOT_READY;
    }
    return ROOT_ERROR;
}

void VisionSendInit(union RUI_U_VISION_SEND*  Send_t,TYPEDEF_VISION *VISION_DATA,User_Data_T*User_Data_aaa,DBUS_Typedef *DBUS_sss,IMU_Data_t *IMU_Data)
{
    static uint8_t buff_flag = 0;
//注意正负
    Send_t->PIT_DATA = -IMU_Data ->pitch ;     // @note c板侧放，如果想用pitch建议改imu_temp...c中的IMU_QuaternionEKF_Update参数顺序和正负
	
    Send_t->YAW_DATA =  -IMU_Data ->yaw ;
	Send_t->ROLL_DATA =IMU_Data ->roll ;
	
	
	
	
    Send_t->INIT_FIRING_RATE =User_Data_aaa->shoot_data.initial_speed;
	
    Send_t->FLAG = VISION_DATA->SEND.FLAG;
    Send_t->COLOR = VISION_DATA->SEND.COLOR;
    Send_t->TIME = VISION_DATA->SEND.TIME;
	//发弹速的
    Send_t->bulletSpeed = 20;//(uint8_t)User_Data_aaa->shoot_data.initial_speed;

    if (DBUS_sss->KeyBoard .B && !DBUS_sss->KeyBoard .B_PreeNumber ) // 按下B键
        buff_flag = 2;//!buff_flag; // 切换打小符模式
//		if (DBUS_sss->KeyBoard .R  && !DBUS_sss->KeyBoard .R_PreeNumber)//暂定大符模式是R
//				buff_flag = 3;      //大符模式
    DBUS_sss->KeyBoard .B_PreeNumber  =DBUS_sss->KeyBoard .B ;
//		DBUS_sss->KeyBoard .R_PreeNumber  =DBUS_sss->KeyBoard .R ;
    Send_t->is_buff = buff_flag;
}

/// @brief 视觉发送
/// @param Send_t 发送变量
/// @param buff 发送处理数据
/// @param type 类型 0:虚拟串口 1:USART1
/// @return 
int ControltoVision(union RUI_U_VISION_SEND*  Send_t , uint8_t *buff, uint8_t type,User_Data_T *users,DBUS_Typedef*DBUS,IMU_Data_t *IMU_Data,TYPEDEF_VISION *VISION_V_DATA)
{
    uint8_t status;
   VisionSendInit(Send_t,VISION_V_DATA,users,DBUS,IMU_Data);
    buff[0] = 0xcd;
	//pitch
	data_tackle.F = Send_t->PIT_DATA;
	buff[1] = data_tackle.U[0];
	buff[2] = data_tackle.U[1];
	buff[3] = data_tackle.U[2];
	buff[4] = data_tackle.U[3];
	//yaw
	data_tackle.F = Send_t->YAW_DATA;
	buff[5] = data_tackle.U[0];
	buff[6] = data_tackle.U[1];
	buff[7] = data_tackle.U[2];
	buff[8] = data_tackle.U[3];
	//roll
//	data_tackle.F =Send_t->ROLL_DATA ;
//	buff[9] =data_tackle.U [0];
//	buff[10]=data_tackle.U [1];
//	buff[11]=data_tackle.U [2];
//	buff[12]=data_tackle.U [3];
//    //将请求的状态置于第九位中
	//2023-06-02 22:54 | 自瞄/打符标志位
    // setbit(&buff[9], 0, Send_t->COLOR &0x01);
    // //2023-06-02 22:54 | 颜色
	// setbit(&buff[9] , 3 , Send_t->COLOR >> 4);
	
	
    // 自瞄1  打小符2  大符3
    buff[9] = Send_t->is_buff;
    data_tackle.I = (uint32_t)Send_t->TIME; // 视觉自瞄和能量机关切换标志位
	buff[10] = data_tackle.U[0];
	buff[11] = data_tackle.U[1];
	buff[12] = data_tackle.U[2];
	buff[13] = data_tackle.U[3];
    buff[14] = Send_t->bulletSpeed;
    buff[15] = 0xdc;
//49    57
    if (type == 0)
        status = CDC_Transmit_FS(buff, 20);
    else if (type == 1)
        status = HAL_UART_Transmit_DMA(&huart1, buff, 16);

    return ROOT_READY;
}

int errcount = 0;
void VISION_F_Monitor(TYPEDEF_VISION *VISION_V_DATA)
{
    VISION_V_DATA->RECV_FLAG[LAST] = VISION_V_DATA->RECV_FLAG[NOW]; // 上一帧数据

    if (VISION_V_DATA->RECV_FLAG[NOW] == ROOT_ERROR)
    {
        VISION_V_DATA->RECV_OutTime++;
        if (VISION_V_DATA->RECV_OutTime >= 1000000)  // 防止越界
        {
            VISION_V_DATA->RECV_OutTime = 500;
        }
        
    }
    if (VISION_V_DATA->RECV_FLAG[NOW] == ROOT_READY)
    {
        VISION_V_DATA->RECV_OutTime = 0;
    }

    VisionMonitor[VISION_Monitor_IOTA++] = VISION_V_DATA->RECEIVE.PIT_DATA;
    if (VISION_Monitor_IOTA >= (VISION_D_MONITOR_LEN - 1))
    {
        VISION_Monitor_IOTA = 0;
    }
    
    int err = 0;
    for (int i = 1; i < VISION_D_MONITOR_LEN; i++)
    {
        if(VisionMonitor[i] == VisionMonitor[i - 1])
            err++;
    }
    
    errcount = err;
      
    if ((VISION_V_DATA->RECV_OutTime >= 500) || (err >= (VISION_D_MONITOR_LEN - 3))) // 500ms
    {
        VISION_V_DATA->RECV_FLAG[NOW] = ROOT_ERROR;
        
    }
}
TYPEDEF_TOP TOP = {0};

// 本代码根据瑞的代码修改而来
void TOP_F_Cal(void *TOP, const uint8_t *DATA)
{
    TYPEDEF_TOP_DATA *TOP_DATA = (TYPEDEF_TOP_DATA *)TOP;
    union TYPEDEF_TOP_DATA_UNION TOP_DATA_UNION = {0};

    // memcpy(TOP_DATA_UNION.DATA, DATA, sizeof(TOP_DATA_UNION.DATA));

    TOP_DATA->PIT_ANGLE[LAST] = TOP_DATA->PIT_ANGLE[NOW];
    TOP_DATA->YAW_ANGLE[LAST] = TOP_DATA->YAW_ANGLE[NOW];

    // TOP_DATA->PIT_ANGLE[NOW] = TOP_DATA_UNION.
}

float currentAngle = 0.0f;

void TOP_T_Cal()
{
    if (TOP.yaw[4] == 1.0f)
    {
        convertAngleToIndex(yaw, &TOP.yaw[NOW]);
        convertAngleToIndex(pitch, &TOP.pitch[NOW]);
        TOP.yaw[NOW] = TOP.yaw[NOW] + currentAngle;
    }
    
    if (TOP.yaw[NOW] - TOP.yaw[LAST] > 4096)
    {
        TOP.yaw[2]--;
    }
    else if (TOP.yaw[NOW] - TOP.yaw[LAST] < -4096)
    {
        TOP.yaw[2]++;
    }
    TOP.yaw[3] = TOP.yaw[2] * 8192.0f + TOP.yaw[NOW];
    TOP.yaw[LAST] = TOP.yaw[NOW];

    TOP.pitch[5] = pitch;
    TOP.yaw[5] = yaw;
    TOP.roll[5] = -roll;
}

void TOP_T_Monitor(DJI_MOTOR_Typedef *ALL_MOTOR)/////////////////这个函数还没加，若预测飘加上
{
    if (yaw == 0.0f && pitch == 0.0f)
    {
        TOP.yaw[4] = 0.0f; // 0-close, offline
    }
    else
    {
        TOP.yaw[4] = 1.0f; // 1-open, online
    }

    if (TOP.yaw[4] == 0.0f) // offline
    {
        // top to motor angle
        TOP.yaw[NOW] = (float)ALL_MOTOR->DATA .Angle_now ;
        TOP.yaw[2] = 0;
        currentAngle = 0;
    }
}

void TOP_T_Cal_T()
{
    if (TOP.yaw[4] == 1.0f)
    {
        convertAngleToIndex(yaw, &TOP.yaw[NOW]);
        convertAngleToIndex(pitch, &TOP.pitch[NOW]);
        TOP.yaw[NOW] = TOP.yaw[NOW] + currentAngle;
    }
				
    if (TOP.yaw[NOW] - TOP.yaw[LAST] < 4096)
    {
        TOP.yaw[2]++;
    }
    else if (TOP.yaw[NOW] - TOP.yaw[LAST] > -4096)
    {
        TOP.yaw[2]--;
    }
    TOP.yaw[3] = TOP.yaw[2] * 8192.0f + TOP.yaw[NOW];
    TOP.yaw[LAST] = TOP.yaw[NOW];

    TOP.pitch[5] = pitch;
    TOP.yaw[5] = yaw;
}
