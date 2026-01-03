#include "Chassis_Task.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR)
{
    //发射电机初始化
    float PID_F_1[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_1[3] = {   2.0f,   0.0f,   0.0f   };

	  float PID_F_2[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   2.0f,   0.0f,   0.0f   };

		float PID_F_3[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   2.0f,   0.0f,   0.0f   };

		float PID_F_4[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   2.0f,   0.0f,   0.0f   };

//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_1.PID_F, 3000, PID_F_1,
//                     0.5f, 2, 2);
//		
//	  Feedforward_Init(&MOTOR->DJI_3508_Chassis_2.PID_F, 3000, PID_F_2,
//                     0.5f, 2, 2);

//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_3.PID_F, 3000, PID_F_3,
//                     0.5f, 2, 2);

//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_4.PID_F, 3000, PID_F_4,
//                     0.5f, 2, 2);

		if(IMU_Data .pitch >-40&&IMU_Data .pitch <5)
{

    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 100000.0f, 1000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 100000.0f, 1000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 100000.0f, 1000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 100000.0f, 1000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
	        }
			
					//上坡pid
			else if(IMU_Data .pitch >=5 &&IMU_Data .pitch <30)
			{
	  float PID_F_1[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_1[3] = {   10.0f,   0.0f,   0.0f   };
    float PID_F_2[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   10.0f,   0.0f,   0.0f   };
    float PID_F_3[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   10.0f,   0.0f,   0.0f   };
    float PID_F_4[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   10.0f,   0.0f,   0.0f   };

//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_1.PID_F, 3000, PID_F_1,
//                     0.5f, 2, 2);
//		Feedforward_Init(&MOTOR->DJI_3508_Chassis_2.PID_F, 3000, PID_F_2,
//                     0.5f, 2, 2);
//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_3.PID_F, 3000, PID_F_3,
//                     0.5f, 2, 2);
//    Feedforward_Init(&MOTOR->DJI_3508_Chassis_4.PID_F, 3000, PID_F_4,
//                     0.5f, 2, 2);

    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 20000.0f, 1000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 20000.0f, 1000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 20000.0f, 1000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 20000.0f, 1000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

			}
    return RUI_DF_READY;
}

uint8_t Chassis_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_Chassis_1     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_2     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_3     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_4     == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    //电机清空
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_1, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_2, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_3, 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_4, 1);

    return RUI_DF_READY;
}

uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   User_Data_T *User_data,
                   model_t *model,
                   CAP_RXDATA *CAP_GET,
                   MOTOR_Typdef *MOTOR)
{
    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Chassis_INIT(MOTOR);
        return RUI_DF_ERROR;
    }

    /*电机在线检测*/
//    if (AIM_INIT != RUI_DF_READY)
//    {
//        AIM_INIT = Chassis_AIM_INIT(Root, MOTOR);
//        return RUI_DF_ERROR;
//    }

		
		
		//飞坡检测
		
//    /*目标值赋值*/
//		if(IMU_Data .pitch >-40&&IMU_Data .pitch <5)
//		{
    MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel1*1.0f;
    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel2*1.0f;
    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel3*1.0f;
    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel4*1.0f;
//  	}
//		else if(IMU_Data .pitch >=5&&IMU_Data .pitch <40)
//		{
//	  MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel1*3.0f;
//    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel2*3.0f;
//    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel3*5.0f;
//    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel4*5.0f;
//		}
    /*遥控离线保护*/
   if(!Root->RM_DBUS)
    {
        CONTAL->BOTTOM.wheel1 = 0;
        CONTAL->BOTTOM.wheel2 = 0;
        CONTAL->BOTTOM.wheel3 = 0;
        CONTAL->BOTTOM.wheel4 = 0;

        PID_INIT = RUI_DF_ERROR;
        AIM_INIT = RUI_DF_ERROR;
    }

    /*堵转处理*/
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_3508_Chassis_1, 4000, 10);
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_3508_Chassis_2, 4000, 10);
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_3508_Chassis_3, 4000, 10);
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_3508_Chassis_4, 4000, 10);
    /*Chassis_1*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_F,
//                          MOTOR->DJI_3508_Chassis_1.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_1.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_1.DATA.Aim);

    /*Chassis_2*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_F,
//                          MOTOR->DJI_3508_Chassis_2.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_2.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_2.DATA.Aim);

    /*Chassis_3*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_F,
//                          MOTOR->DJI_3508_Chassis_3.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_3.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_3.DATA.Aim);

    /*Chassis_4*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Chassis_4.PID_F,
//                          MOTOR->DJI_3508_Chassis_4.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_4.PID_S,
                  (float)MOTOR->DJI_3508_Chassis_4.DATA.Speed_now,
                  MOTOR->DJI_3508_Chassis_4.DATA.Aim);
    /*电机动力分配*/

    /*功率控制*/

    return RUI_DF_READY;
}
