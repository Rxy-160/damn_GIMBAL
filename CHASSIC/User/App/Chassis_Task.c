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
    float PID_S_1[3] = {   3.0f,   0.0f,   0.0f   };

	  float PID_F_2[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   3.0f,   0.0f,   0.0f   };

		float PID_F_3[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   3.0f,   0.0f,   0.0f   };

		float PID_F_4[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   3.0f,   0.0f,   0.0f   };

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
    float PID_S_1[3] = {   2.0f,   0.0f,   0.0f   };
    float PID_F_2[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   2.0f,   0.0f,   0.0f   };
    float PID_F_3[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   2.0f,   0.0f,   0.0f   };
    float PID_F_4[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   2.0f,   0.0f,   0.0f   };

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
    MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel1*2.0f;
    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel2*2.0f;
    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel3*2.0f;
    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel4*2.0f;
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

//        PID_INIT = RUI_DF_ERROR;
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
//    chassis_power_control(CONTAL,
//                          User_data,
//                          model,
//                          CAP_GET,
//                          MOTOR);

    /*总输出计算*/
    float tmp_C[4];
    tmp_C[0] = /*MOTOR->DJI_3508_Chassis_1.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_1.PID_S.Output;

    tmp_C[1] = /*MOTOR->DJI_3508_Chassis_2.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_2.PID_S.Output;

    tmp_C[2] = /*MOTOR->DJI_3508_Chassis_3.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_3.PID_S.Output;

    tmp_C[3] = /*MOTOR->DJI_3508_Chassis_4.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_4.PID_S.Output;

    /*CAN发送*/
    DJI_Current_Ctrl(&hcan2,
                     0x200,
                     (int16_t)tmp_C[0],
                     (int16_t)tmp_C[1],
                     (int16_t)tmp_C[2],
                     (int16_t)tmp_C[3]);

    return RUI_DF_READY;
}
uint16_t ralati;
/************************************************************万能分隔符**************************************************************
*	@author:			//赵澍
*	@performance:	//底盘接收云台CAN解算
*	@parameter:		//需要解算的数组
*	@time:				//22-01-03
*	@ReadMe:			//接收到的数据放到了底盘结构体里面
								//协议：[0][1] = vx  ||  [2][3] = vy  ||  [4][5] = vr  
												[6] : {1}:是否使用超级电容 {2}：遥控是否在线 {34}：底盘的期望状态
												[7] 空闲
************************************************************************************************************************************/
uint8_t ChassisRXResolve(uint8_t * data,DBUS_Typedef *DBUS,RUI_ROOT_STATUS_Typedef *Root)
{
	CanCommunit_t.gmTOch.getData[0] = data[0];
	CanCommunit_t.gmTOch.getData[1] = data[1];
	CanCommunit_t.gmTOch.getData[2] = data[2];
	CanCommunit_t.gmTOch.getData[3] = data[3];
	CanCommunit_t.gmTOch.getData[4] = data[4];
	CanCommunit_t.gmTOch.getData[5] = data[5];
	CanCommunit_t.gmTOch.getData[6] = data[6];
	CanCommunit_t.gmTOch.getData[7] = data[7];
	//赋值给遥控	
	DBUS->Remote .CH0_int16  = CanCommunit_t.gmTOch.dataNeaten.vx;
	DBUS->Remote .CH1_int16  = CanCommunit_t.gmTOch.dataNeaten.vy;
	DBUS->Remote.Dial_int16  = CanCommunit_t.gmTOch.dataNeaten.vr;
	DBUS->Remote .S1_u8 =CanCommunit_t.gmTOch .dataNeaten .S1 ;
	DBUS->Remote .S2_u8 =CanCommunit_t.gmTOch .dataNeaten .S2 ;
	DBUS->KeyBoard .C  = CanCommunit_t.gmTOch.dataNeaten.key_c;
	DBUS->KeyBoard .Q  = CanCommunit_t.gmTOch.dataNeaten.key_q;
	DBUS->KeyBoard .E  = CanCommunit_t.gmTOch.dataNeaten.key_e;
	DBUS->KeyBoard .Ctrl  = CanCommunit_t.gmTOch.dataNeaten.key_ctrl;
	DBUS->KeyBoard .G  = CanCommunit_t.gmTOch.dataNeaten.key_g;
	DBUS->KeyBoard .F  = CanCommunit_t.gmTOch.dataNeaten.key_f;
	DBUS->KeyBoard .X  = CanCommunit_t.gmTOch.dataNeaten.key_x;
	DBUS->KeyBoard .R  = CanCommunit_t.gmTOch.dataNeaten.key_r;
  RUI_ROOT_STATUS.RM_DBUS =CanCommunit_t .gmTOch .dataNeaten .DBUS_state ;
	
	//陀螺仪检测
//	Root->MASTER_LOCATION  = CanCommunit_t.gmTOch.dataNeaten.topSate;
//	if(CanCommunit_t.gmTOch.dataNeaten.topSate == 1){
//			Root->MASTER_LOCATION = 0;
//	}

	
	return 1;
}


/************************************************************万能分隔符**************************************************************
*	@author:			//赵澍
*	@performance:	//底盘给云台的CAN发送解算函数
*	@parameter:		//存放发送的数组
*	@time:				//22-01-04
*	@ReadMe:			//运行完成之后数据可以直接CAN发送
								//协议：第一个数组
												[12] pitchAngle [34] yawAngle	[4567] 时间轴
												第二个数组
												[12] 枪口热量
												[3] 最大射速
												[4]	当前射速	
												[5]	{1}视觉是否识别到目标 {234} 视觉的当前状态 {5}视觉是否离线 {6}裁判系统是否离线
************************************************************************************************************************************/
uint8_t ChassisTXResolve(User_Data_T *User_data)
{	
		static uint16_t heatlimit = 200;
		if(User_data->robot_status.robot_level == 1){
			heatlimit = 200;
		}else if(User_data->robot_status.robot_level == 2){
			heatlimit = 230;
		}else if(User_data->robot_status.robot_level == 3){
			heatlimit = 260;
		}else if(User_data->robot_status.robot_level == 4){
			heatlimit = 290;
		}else if(User_data->robot_status.robot_level == 5){
			heatlimit = 320;
		}else if(User_data->robot_status.robot_level == 6){
			heatlimit = 350;
		}else if(User_data->robot_status.robot_level == 7){
			heatlimit = 380;
		}else if(User_data->robot_status.robot_level == 8){
			heatlimit = 420;
		}else if(User_data->robot_status.robot_level == 9){
			heatlimit = 450;
		}else if(User_data->robot_status.robot_level ==10){
			heatlimit = 500;
		}
		//新协议
//		if(boll == 0){
//			//第一帧
////			CanCommunit_t.chTOgm.dataNeaten_angle.pitch = SectionLimit_i(30000, -30000, (visionData_t.receive.pitchAngle[df_now] * 100));
////			CanCommunit_t.chTOgm.dataNeaten_angle.yaw 	= SectionLimit_i(30000, -30000, (visionData_t.receive.yawAngle[df_now] * 100));
////			CanCommunit_t.chTOgm.dataNeaten_angle.time	= (float)RunTime;
//		}
//		else {
			//第二帧
			CanCommunit_t.chTOgm.dataNeaten_another.muzzleColing = heatlimit - User_data->power_heat_data.shooter_42mm_barrel_heat;	//剩余枪口热量
			CanCommunit_t.chTOgm.dataNeaten_another.maxSpeed = 16;	//最大射速
			CanCommunit_t.chTOgm.dataNeaten_another.nowSpeed = User_data->shoot_data.initial_speed; //当前射速
//			CanCommunit_t.chTOgm.dataNeaten_another.nowSpeed = 0;		//底盘锁
//			CanCommunit_t.chTOgm.dataNeaten_another.target = visionData_t.receive.target;		//视觉是否识别到目标
//			CanCommunit_t.chTOgm.dataNeaten_another.visionMod = visionData_t.receive.visionState;		//视觉的当前状态
//			CanCommunit_t.chTOgm.dataNeaten_another.visionState = root_t.visionRoot.communicat;		//视觉离线信息
//			CanCommunit_t.chTOgm.dataNeaten_another.judgeState = root_t.judgeRoot.communicat;		//裁判系统离线信息
//		}
//		can_send(&hcan1,df_CHControlData_ID,CanCommunit_t.chTOgm.sendData[0],\
//				CanCommunit_t.chTOgm.sendData[1],\
//				CanCommunit_t.chTOgm.sendData[2],\
//				CanCommunit_t.chTOgm.sendData[3]);
//	DJI_Current_Ctrl(&hcan1,GIMBAL_kong,100,100,100,100);
		return 1;
}
