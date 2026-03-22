#include "Shoot_Task.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t MOTOR_PID_Shoot_INIT(MOTOR_Typdef *MOTOR)
{
    //发射电机初始化
    float PID_F_L[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_L[3] = {   1.0f,   0.0f,   0.0f   };
    Feedforward_Init(&MOTOR->DJI_3508_Shoot_L.PID_F, 3000, PID_F_L,
                     0.5f, 2, 2);
    PID_Init(&MOTOR->DJI_3508_Shoot_L.PID_S, 6000.0f, 2000.0f,
             PID_S_L, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

    float PID_F_R[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_R[3] = {   1.0f,   0.0f,   0.0f   };
    Feedforward_Init(&MOTOR->DJI_3508_Shoot_R.PID_F, 3000, PID_F_R,
                     0.5f, 2, 2);
    PID_Init(&MOTOR->DJI_3508_Shoot_R.PID_S, 6000.0f, 2000.0f,
             PID_S_R, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

    float PID_F_M[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_P_M[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_M[3] = {   1.0f,   0.0f,   0.0f   };
    Feedforward_Init(&MOTOR->DJI_3508_Shoot_M.PID_F, 3000, PID_F_M,
                     0.5f, 2, 2);
    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_P, 6000.0f, 2000.0f,
             PID_P_M, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_S, 6000.0f, 2000.0f,
             PID_S_M, 1000.0f, 1000.0f,
             0.7f, 0.7f, 2,
             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

    return RUI_DF_READY;
}

uint8_t Shoot_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_Shoot_L     == RUI_DF_OFFLINE ||
        Root->MOTOR_Shoot_M     == RUI_DF_OFFLINE ||
        Root->MOTOR_Shoot_R     == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    //电机清空
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_L, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_R, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_M, 1);

    return RUI_DF_READY;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//2024/1/11 14:53
 *	@ReadMe:			//卡弹检测
 ************************************************************万能分隔符**************************************************************/
uint8_t RUI_F_JAM(DJI_MOTOR_DATA_Typedef *DATA, CONTAL_Typedef *CONTAL)
{
    int64_t ERROR_ANGLE = (int64_t) DATA->Aim - DATA->Angle_Infinite;
    //一定误差速度
    if (RUI_F_MATH_ABS_int64_t(ERROR_ANGLE) > 200 && RUI_F_MATH_ABS_int16_t(DATA->Speed_now) < 100)
    {
        //防止是正在拨弹过程中判断为卡弹，加入了时间，在一定的时间内还没有拨弹成功才判断为卡弹
        if ((int64_t)DWT_GetTimeline_ms() - DATA->Stuck_Time > 200000)
        {
            DATA->Aim =  (float) (DATA->Angle_Infinite - CONTAL->SHOOT.Single_Angle);
            return RUI_DF_ERROR;
        }
    }
    else
    {
        //正常拨弹时，可能角度会有误差，但速度不会低于某个值，所以判断为正常拨弹，更新卡弹时间
        DATA->Stuck_Time = (int64_t)DWT_GetTimeline_ms();
        return RUI_DF_READY;
    }
    return RUI_DF_READY;
}

uint8_t shoot_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   MOTOR_Typdef *MOTOR)
{
    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Shoot_INIT(MOTOR);
        return RUI_DF_ERROR;
    }

    /*电机在线检测*/
//    if (AIM_INIT != RUI_DF_READY)
//    {
//        AIM_INIT = Shoot_AIM_INIT(Root, MOTOR);
//        return RUI_DF_ERROR;
//    }

    /*目标值赋值*/
    MOTOR->DJI_3508_Shoot_L.DATA.Aim = CONTAL->SHOOT.SHOOT_L;
    MOTOR->DJI_3508_Shoot_R.DATA.Aim = CONTAL->SHOOT.SHOOT_R;
    MOTOR->DJI_3508_Shoot_M.DATA.Aim = CONTAL->SHOOT.SHOOT_M;

    /*遥控离线保护*/
//    if(!Root->RM_DBUS)
//    {
//        MOTOR->DJI_3508_Shoot_L.PID_S.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_L.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_R.PID_S.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_R.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_M.PID_P.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_M.PID_S.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_M.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite;

//        PID_INIT = RUI_DF_ERROR;
//        AIM_INIT = RUI_DF_ERROR;
//    }

    /*堵转处理*/
//    if(MOTOR->DJI_3508_Shoot_L.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_L, 0);
//    if(MOTOR->DJI_3508_Shoot_R.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_R, 0);
//    if(MOTOR->DJI_3508_Shoot_M.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_M, 0);

    /*Shoot_L*/
    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_F,
                          MOTOR->DJI_3508_Shoot_L.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now,
                  MOTOR->DJI_3508_Shoot_L.DATA.Aim);

    /*Shoot_R*/
    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_F,
                          MOTOR->DJI_3508_Shoot_R.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now,
                  MOTOR->DJI_3508_Shoot_R.DATA.Aim);

    /*Shoot_M*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_F,
//                          MOTOR->DJI_3508_Shoot_M.DATA.Aim);
//    PID_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_P,
//                  (float)MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite,
//                  MOTOR->DJI_3508_Shoot_M.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_M.DATA.Speed_now,
                  /*MOTOR->DJI_3508_Shoot_M.PID_P.Output*/12000);

    /*总输出计算*/
    float tmp_S[3];

    tmp_S[0] = MOTOR->DJI_3508_Shoot_L.PID_F.Output +
               MOTOR->DJI_3508_Shoot_L.PID_S.Output;

    tmp_S[1] = MOTOR->DJI_3508_Shoot_R.PID_F.Output +
               MOTOR->DJI_3508_Shoot_R.PID_S.Output;

    tmp_S[2] =/* MOTOR->DJI_3508_Shoot_M.PID_F.Output +*/
               MOTOR->DJI_3508_Shoot_M.PID_S.Output;

    /*CAN发送*/
//    DJI_Current_Ctrl(&hcan1,
//                     0x200,
//                     (int16_t)tmp_S[0],
//                     (int16_t)tmp_S[1],
//                     (int16_t)tmp_S[2],
//                     0);

    /*发射机构信息反馈*/
    CONTAL->SHOOT_Bask.Speed_Aim_L = MOTOR->DJI_3508_Shoot_L.DATA.Aim;
    CONTAL->SHOOT_Bask.Speed_Aim_R = MOTOR->DJI_3508_Shoot_R.DATA.Aim;
    CONTAL->SHOOT_Bask.Speed_err_L = MOTOR->DJI_3508_Shoot_L.DATA.Aim -
                                     (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now;
    CONTAL->SHOOT_Bask.Speed_err_R = MOTOR->DJI_3508_Shoot_R.DATA.Aim -
                                     (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now;
    CONTAL->SHOOT_Bask.Angle = MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite;
    CONTAL->SHOOT_Bask.JAM_Flag = RUI_F_JAM(&MOTOR->DJI_3508_Shoot_M.DATA, CONTAL);
    CONTAL->SHOOT_Bask.Shoot_Number = MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite / CONTAL->SHOOT.Single_Angle;

    return RUI_DF_READY;
}

uint8_t bodan_state=0;
void shoot_state(MOTOR_Typdef *MOTOR , 
	               DBUS_Typedef *DBUS)
{
	if(DBUS->Remote .S2_u8 ==2)
	{
		
		if (DBUS->Remote .S1_u8==3)
		{
			bodan_state=0;
		}
		else if (DBUS->Remote .S1_u8 ==1)
		{
			bodan_state=1;
		}
		else if(DBUS->Remote .S1_u8 ==2)
		{
			bodan_state=2;
		}
	}
}
//void supply_state