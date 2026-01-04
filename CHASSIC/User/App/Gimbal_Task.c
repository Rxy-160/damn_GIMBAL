#include "Gimbal_Task.h"
#include "VOFA.h"
    float tmp_G[2];
int16_t target;
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
 
 
uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typdef *MOTOR)
{
    //云台电机初始化
    float PID_F_Pitch[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_P_Pitch[3] = {   1.0,   0.0f,   0.0f   };
    float PID_S_Pitch[3] = {   /*150.0f*/180,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_6020_Pitch.PID_F, 3000, PID_F_Pitch,
//                     0.5f, 2, 2);
		
    PID_Init(&MOTOR->DJI_6020_Pitch.PID_P, 30000.0f, 2000.0f,
             PID_P_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_6020_Pitch.PID_S, 30000.0f, 2000.0f,
             PID_S_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             );//微分先行,微分滤波器

    float PID_F_Yaw[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_P_Yaw[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_Yaw[3] = {   /*160.0f*/180,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_6020_Yaw.PID_F, 3000, PID_F_Yaw,
//                     0.5f, 2, 2);
//		
    PID_Init(&MOTOR->DJI_6020_Yaw.PID_P, 30000.0f, 2000.0f,
             PID_P_Yaw, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_6020_Yaw.PID_S, 30000.0f, 2000.0f,
             PID_S_Yaw, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    return RUI_DF_READY;
}

uint8_t Gimbal_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
        Root->MOTOR_HEAD_Yaw      == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    //电机清空
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Pitch, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Yaw, 1);

    return RUI_DF_READY;
}

uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    RUI_ROOT_STATUS_Typedef *Root,
                    MOTOR_Typdef *MOTOR,
                    IMU_Data_t *IMU)
{
	
    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

	////////这个千万不能注释
    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
      PID_INIT = MOTOR_PID_Gimbal_INIT(MOTOR);
      return RUI_DF_ERROR;
    }
///////这个必须注释
    /*电机在线检测*/
//    if (AIM_INIT != RUI_DF_READY)
//    {
//      AIM_INIT = Gimbal_AIM_INIT(Root, MOTOR);
//      return RUI_DF_ERROR;
//    }
		
///底盘跟随记得改回来
		
    /*底盘跟随变量赋值*/
    CONTAL->CG.RELATIVE_ANGLE = -(int16_t) (CONTAL->CG.YAW_INIT_ANGLE - MOTOR->DJI_6020_Yaw.DATA.Angle_now);
//    CONTAL->CG.RELATIVE_ANGLE = (int16_t) ( (IMU->YawTotalAngle * 22.75555556f/*22.75555555555556f*/ )- CONTAL->HEAD.Yaw );

    CONTAL->CG.YAW_SPEED =  MOTOR->DJI_6020_Yaw.DATA.Speed_now;

    /*目标值赋值*/

    return RUI_DF_READY;
}
