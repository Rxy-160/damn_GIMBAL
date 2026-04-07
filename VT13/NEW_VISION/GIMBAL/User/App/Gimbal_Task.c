#include "Gimbal_Task.h"
#include "VOFA.h"
    float tmp_G[2];
		float abcd;
int16_t target;
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //头部PID+前馈总初始化函数
 *	@parameter:		    //
 *	@time:				//23-04-13 12:42
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/

uint8_t MOTOR_PID_Gimbal_INIT(MOTOR_Typdef *MOTOR,TD_t *TDDD)
{
//
	TD_Init(TDDD, abcd, 0.003);
    //云台电机初始化
//    float PID_F_Pitch[3] = {   0.0f,   0.0f,   0.0f   };
//    float PID_P_Pitch[3] = {   2.0f,   1.3f,   0.0f   };//{   2.0f,   0.004f,   0.0f   };
//    float PID_S_Pitch[3] = {   /*150.0f*/67.0f,   0.3f,   0.0f   };//{   /*150.0f*/35.0f,   0.001f,   0.0f   };
	  float PID_F_Pitch[3] = {   0.0f,   0.0f,   0.0f   };
//    float PID_P_Pitch[3] = {   3.0f,   0.1f,   0.0f   };//{   2.0f,   0.004f,   0.0f   };
//    float PID_S_Pitch[3] = {   /*150.0f*/67.0f,   0.3f,   0.0f   };//{   /*150.0f*/35.0f,   0.001f,   0.0f   };
    float PID_P_Pitch[3] = {   3.0f,   1.0f,   0.0f   };//{   2.0f,   0.004f,   0.0f   };
    float PID_S_Pitch[3] = {   /*150.0f*/40.0f,   0.3f,   0.0f   };//{   /*150.0f*/35.0f,   0.001f,   0.0f   };

		///////////////////////////////////////70
//    Feedforward_Init(&MOTOR->m_dm4310_p_t .PID_F, 3000, PID_F_Pitch,
//                     0.5f, 2, 2);
		//
    PID_Init(&MOTOR->m_dm4310_p_t .PID_P, 12000.0f, 800.0f,
             PID_P_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->m_dm4310_p_t .PID_S, 30000.0f, 3000.0f,
             PID_S_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             );//微分先行,微分滤波器

//    float PID_F_Yaw[3] = {   0.0f,   0.0f,   0.0f   };
//    float PID_P_Yaw[3] = {   2.6f,   4.0f,   0.0f   };//{   2.0f,   0.8f,   0.0f  };
//    float PID_S_Yaw[3] = {   /*160.0f*/60,   0.1f,   0.3f   };//{   /*160.0f*/100,   0.0f,   0.0f    };
		    float PID_F_Yaw[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_P_Yaw[3] = {   3.0f,   1.5f,   0.0f   };//{   2.0f,   0.8f,   0.0f  };
    float PID_S_Yaw[3] = {   /*160.0f*/45,   0.1f,   0.3f   };//{   /*160.0f*/100,   0.0f,   0.0f    };

		////////////////////////////78
//    Feedforward_Init(&MOTOR->m_dm4310_y_t .PID_F, 3000, PID_F_Yaw,
//                    0.5f, 2, 2);
//		
		
    PID_Init(&MOTOR->m_dm4310_y_t  .PID_P , 1000.0f, 80.0f,
             PID_P_Yaw, 1000, 1000,
             0, 0.5, 0,
             Integral_Limit|ErrorHandle|ChangingIntegrationRate|Trapezoid_Intergral|OutputFilter|DerivativeFilter//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->m_dm4310_y_t .PID_S, 30000.0f, 500.0f,
             PID_S_Yaw, 0, 0,
             0, 0.2, 0,
             Integral_Limit|ErrorHandle|ChangingIntegrationRate|Trapezoid_Intergral|OutputFilter|DerivativeFilter//积分限幅,输出滤波,堵转监测
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
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Yaw  , 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Pitch , 1);

    return RUI_DF_READY;
}
int a=0;
uint8_t gimbal_task(CONTAL_Typedef *CONTAL,
                    RUI_ROOT_STATUS_Typedef *Root,
                    MOTOR_Typdef *MOTOR,
                    IMU_Data_t *IMU,
                    TD_t *TDDDD
                    )
{
	////////3508速度传递放在这
	MOTOR->DJI_3508_Shoot_L .DATA .Speed_last =MOTOR->DJI_3508_Shoot_L .DATA .Speed_now ;
	MOTOR->DJI_3508_Shoot_R .DATA .Speed_last =MOTOR->DJI_3508_Shoot_R .DATA .Speed_now ;
		MOTOR->DJI_3508_Shoot_M .DATA .Speed_last =MOTOR->DJI_3508_Shoot_M .DATA .Speed_now ;

    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

	////////这个千万不能注释
    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
      PID_INIT = MOTOR_PID_Gimbal_INIT(MOTOR,TDDDD);
      return RUI_DF_ERROR;
    }
///////这个必须注释
    /*电机在线检测*/
//    if (AIM_INIT != RUI_DF_READY)
//    
//      AIM_INIT = Gimbal_AIM_INIT(Root, MOTOR);
//      return RUI_DF_ERROR;
//    
		
		/////VOFA调用
								//				VOFA_justfloat(IMU->gyro_correct [0] ,
//		               IMU->gyro_correct [1]  ,
//		               IMU->gyro_correct [2]  ,
//		               IMU->YawTotalAngle,
//		               0 ,
//		               0,0,0,0,0);

///底盘跟随记得改回来

    /*底盘跟随变量赋值*/
//    CONTAL->CG.RELATIVE_ANGLE = (int16_t) (CONTAL->CG.YAW_INIT_ANGLE  - MOTOR->m_dm4310_y_t .DATA .Angle_now);//////有yaw的车
		
//    CONTAL->CG.RELATIVE_ANGLE = (int16_t) ( (IMU->YawTotalAngle * 22.75555555555556f )- CONTAL->HEAD.Yaw );

    CONTAL->CG.YAW_SPEED =  MOTOR->m_dm4310_y_t .DATA.Speed_now;
    if (CONTAL->CG.RELATIVE_ANGLE > 4096)
    {
        CONTAL->CG.RELATIVE_ANGLE -= 8192;
    }
    else if (CONTAL->CG.RELATIVE_ANGLE < -4096)
    {
        CONTAL->CG.RELATIVE_ANGLE += 8192;
    }
//150   -150
    /*目标值赋值*/

    MOTOR->m_dm4310_p_t .DATA.Aim = CONTAL->HEAD.Pitch;
    MOTOR->m_dm4310_y_t .DATA.Aim = CONTAL->HEAD.Yaw;

//    if(/*(CONTAL->MOD[0] - CONTAL->MOD[1] == 1*/CONTAL->MOD[0]==0)//自瞄模式/////手瞄到自瞄
//    {
//        float PID_P_Yaw_vision[3] = {   1.9f,   0.2f,   0.0f   };
//        float PID_S_Yaw_vision[3] = {   80.0f ,   0.3f,   0.0f   };
//				float PID_P_Pitch_vision[3] = {   1.8f,   0.00f,   0.0f   };
//        float PID_S_Pitch_vision[3] = {   20.0f,   0.00f,   0.0f   };
//	//////////这个函数的目的是把不同模式的pid给赋进去
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_P, PID_P_Yaw_vision);
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_S, PID_S_Yaw_vision);
//				PID_set(&MOTOR->m_dm4310_p_t .PID_P, PID_P_Pitch_vision);
//        PID_set(&MOTOR->m_dm4310_p_t .PID_S, PID_S_Pitch_vision);
//    }
//    else if(/*CONTAL->MOD[0] - CONTAL->MOD[1] <= 0*/CONTAL->MOD[0]==1 )//手瞄模式///////自瞄到手瞄
//    {
//        float PID_P_Yaw_hand[3] = {    1.9f,   0.2f,   0.0f  };
//        float PID_S_Yaw_hand[3] = {    80 ,   0.3f,   0.0f   };
//				float PID_P_Pitch_hand[3] = {   1.8f,   0.00f,   0.0f    };
//        float PID_S_Pitch_hand[3] = {  20.0f,   0.00f,   0.0f  };
//	//////////这个函数的目的是把不同模式的pid给赋进去
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_P, PID_P_Yaw_hand);
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_S, PID_S_Yaw_hand);
//				PID_set(&MOTOR->m_dm4310_p_t  .PID_P, PID_P_Pitch_hand);
//        PID_set(&MOTOR->m_dm4310_p_t  .PID_S, PID_S_Pitch_hand);
//    }
//纯过零解算
			

    /*遥控离线保护*/
    if(!Root->RM_DBUS)
    {
//			  MOTOR->m_dm4310_p_t  .PID_P .IntegralLimit =0;
//			  MOTOR->m_dm4310_p_t  .PID_S .IntegralLimit =0;
        MOTOR->m_dm4310_p_t .DATA.Aim = (float)IMU->pitch *22.7555556f - VT13_DBUS.Mouse.Y_Flt * 0.7f;
			
//			  MOTOR->m_dm4310_y_t  .PID_P .IntegralLimit =0;
//			  MOTOR->m_dm4310_y_t  .PID_S .IntegralLimit =0;
        MOTOR->m_dm4310_y_t .DATA.Aim = (float)IMU->YawTotalAngle*22.7555556f -(RUI_F_MATH_Limit_float(1, -1, VT13_DBUS.Mouse.X_Flt * 1.3f) +(float) (VT13_DBUS.KeyBoard.E - VT13_DBUS.KeyBoard.Q)*2.8);
			
//				WHW_V_DBUS.Remote .S1_u8 =0;
//				WHW_V_DBUS.Remote .S2_u8 =0;
//			dm4310_current_set(&hcan1,0x3FE,0,0,0,0);
//			DJI_Current_Ctrl(&hcan1,0x200,0,0,0,0);
//	    DJI_Current_Ctrl(&hcan2,0x200,0,0,/*(int16_tfloat)tmp_S[2]*/0,0);

//        PID_INIT = RUI_DF_ERROR;
        AIM_INIT = RUI_DF_ERROR;
    }
    /*堵转处理*/
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_6020_Pitch, 4000, 10);
//    RUI_F_HEAD_MOTOR3508_STUCK(&MOTOR->DJI_6020_Yaw, 4000, 10);

    /*Pitch计算*/
//    Feedforward_Calculate(&MOTOR->DJI_6020_Pitch.PID_F,
//                          MOTOR->DJI_6020_Pitch.DATA.Aim);
    PID_Calculate(&MOTOR->m_dm4310_p_t .PID_P,
                  IMU->pitch * 22.75555555555556f,
                  MOTOR->m_dm4310_p_t .DATA.Aim);
    PID_Calculate(&MOTOR->m_dm4310_p_t .PID_S,
                 IMU->gyro[1] *100.0f,
                  MOTOR->m_dm4310_p_t.PID_P.Output);

    /*Yaw计算*/
//    Feedforward_Calculate(&MOTOR->DJI_6020_Yaw.PID_F,
//                          MOTOR->DJI_6020_Yaw.DATA.Aim);
//		if(CONTAL->MOD[0] ==0)//手控模式
//		{
    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_P,
                  IMU->YawTotalAngle * 22.75555555555556f,
                  MOTOR->m_dm4310_y_t .DATA.Aim);
    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_S,
                  IMU->gyro[2] * 100.0f,
                  MOTOR->m_dm4310_y_t .PID_P.Output);
//		}
//		else //视觉模式
//		{
//			    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_P,
//                  IMU->yaw  * 22.75555555555556f,
//                  MOTOR->m_dm4310_y_t .DATA.Aim);
//    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_S,
//                  IMU->gyro[2] * 100.0f,
//                  MOTOR->m_dm4310_y_t .PID_P.Output);

//		}

    /*总输出计算*/

    tmp_G[0] = /*MOTOR->m_dm4310_y_t .PID_F.Output*/0   +
               MOTOR->m_dm4310_y_t  .PID_S.Output;

    tmp_G[1] =/* MOTOR->m_dm4310_p_t .PID_F.Output*/ +
               MOTOR->m_dm4310_p_t .PID_S.Output;
							 
		dm4310_current_set(&hcan1,0x3FE,tmp_G[0],tmp_G[1]/*-COS_pitch()*/,0,0);

    /*CAN发送*/
//dm4310_current_set(&hcan1,0x4FE,0,0,0,tmp_G[0]);
//dm4310_current_set(&hcan1,0x3FE,0,0,0,tmp_G[0]);

//mit_ctrl(&hcan1, 2, 0, 0, 0, 0, ALL_MOTOR .m_dm4310_p_t .PID_S .Output  );

    return RUI_DF_READY;
}
/**
  * @author: 楠
  * @performance: 解算从底盘发送来的CAN数据
  * @parameter: @存放接收的数组
  * @time: 23-7-9
  * @ReadMe: 
*/
uint8_t GimbalRXResolve(uint8_t * buff,uint16_t CANID) 
{
	

		//接收信息
		CanCommunit_t.chTOgm.getData[0] = buff[0];
		CanCommunit_t.chTOgm.getData[1] = buff[1];
		CanCommunit_t.chTOgm.getData[2] = buff[2];
		CanCommunit_t.chTOgm.getData[3] = buff[3];
		CanCommunit_t.chTOgm.getData[4] = buff[4];
		CanCommunit_t.chTOgm.getData[5] = buff[5];
		CanCommunit_t.chTOgm.getData[6] = buff[6];
		CanCommunit_t.chTOgm.getData[7] = buff[7];
		
	
	
	
	
	
//剩余热量
	heat_state=CanCommunit_t.chTOgm .dataNeaten_another .heat_last ;
	//缓冲热量
	huanchongnengliang =CanCommunit_t.chTOgm .dataNeaten_another .huanchongnengliang ;
		return 0;
}

/**
  * @author: 楠
  * @performance: 云台发送给底盘的CAN解算函数
  * @parameter: @是否使用超电 @遥控是否在线 @底盘模式 @陀螺仪状态
  * @time: 23-7-9
  * @ReadMe: 协议：[0][1] = vx  ||  [2][3] = vy  ||  [4][5] = vr  
												[6] : {1}:是否使用超级电容 {2}：遥控是否在线 {34}：底盘的期望状态 {5678}：陀螺仪数据高四位
												[7] 陀螺仪数据低八位
*/
uint8_t GimbalTXResovle( VT13_Typedef *VT13_DBUS) 
{
	
//		int16_t pitchAngle = (int16_t)SectionLimit_f(500.0f, -500.0f, (TopData_t.pitchAgnle_f * 10.0f) );
		
//		KeyboardResolve();		//键盘模式底盘速度的解算
			VOFA_justfloat(filler_motor_L    /*ALL_MOTOR .m_dm4310_p_t .DATA .Aim*/   ,
											filler_motor_R  /**22.75555555555*/  ,
		               filler_motor_M     /*ALL_MOTOR .m_dm4310_y_t .DATA .Aim*/  ,
		              ALL_MOTOR.DJI_3508_Shoot_L .DATA .Speed_now    ,
		              ALL_MOTOR.DJI_3508_Shoot_L  .DATA .Speed_last  ,
		              ALL_MOTOR .DJI_3508_Shoot_M .DATA .Speed_now   ,
									asdf,
									 /*IMU_Data.pitch*/statues,
		               motor_M_state  ,
									 /*ALL_MOTOR.m_dm4310_p_t .DATA .Angle_now*/motor_F_state  );/*反馈电流是cur_int16*/

		CanCommunit_t.gmTOch.dataNeaten.vx =  VT13_DBUS->Remote .Channel[0] +(VT13_DBUS->KeyBoard .D -VT13_DBUS->KeyBoard .A )*330;//1
//		CanCommunit_t.gmTOch.dataNeaten.vx += (DBUS->KeyBoard .W -DBUS->KeyBoard .S )*660;//gimbal_t.Keyboard.vx;//键鼠，还没加
		CanCommunit_t.gmTOch.dataNeaten.vy =  VT13_DBUS->Remote .Channel[1] +(VT13_DBUS->KeyBoard .W -VT13_DBUS->KeyBoard .S )*330;//2
//		CanCommunit_t.gmTOch.dataNeaten.vy += (DBUS->KeyBoard .D -DBUS->KeyBoard .A )*660;//gimbal_t.Keyboard.vy;//
		CanCommunit_t.gmTOch.dataNeaten.vr =  ((VT13_DBUS->Remote .wheel)-VT13_DBUS->KeyBoard .Shift *660);//3
//		CanCommunit_t.gmTOch.dataNeaten.vr += DBUS->KeyBoard .Shift *660;//gimbal_t.Keyboard.vr;//
				
		//键位赋值
		CanCommunit_t.gmTOch.dataNeaten.key_f  = VT13_UNION.DataNeaten .KeyBoard_F ;
		CanCommunit_t.gmTOch.dataNeaten.key_g  = VT13_UNION.DataNeaten .KeyBoard_G ;
		CanCommunit_t.gmTOch.dataNeaten.key_c  = VT13_UNION.DataNeaten .KeyBoard_C ;
	    CanCommunit_t.gmTOch.dataNeaten.key_shift =VT13_UNION.DataNeaten .KeyBoard_Shift ;
	    CanCommunit_t.gmTOch.dataNeaten.key_q =VT13_UNION.DataNeaten .KeyBoard_Q ;
		CanCommunit_t.gmTOch.dataNeaten.key_e =VT13_UNION.DataNeaten .KeyBoard_E  ; //不规范写法 后边改过来
		CanCommunit_t.gmTOch.dataNeaten.key_ctrl  = VT13_UNION.DataNeaten .KeyBoard_Ctrl ;
//		CanCommunit_t.gmTOch.dataNeaten.key_r = DBUS->KeyBoard .R_PreeNumber ;
//		CanCommunit_t.gmTOch.dataNeaten.key_ctrl = VT13_UNION.DataNeaten .KeyBoard_Ctrl ;
				CanCommunit_t .gmTOch .dataNeaten .romoteOnLine =RUI_ROOT_STATUS.RM_DBUS ;

		CanCommunit_t.gmTOch .dataNeaten .S1 =VT13_DBUS->Remote .fn_1 ;
		CanCommunit_t.gmTOch .dataNeaten .S2 =VT13_DBUS->Remote .fn_2 ;

		CanCommunit_t.gmTOch.dataNeaten.supUSe = VT13_DBUS->KeyBoard.C ;
		CanCommunit_t.gmTOch .dataNeaten .pitch =IMU_Data.pitch;
		CanCommunit_t.gmTOch.dataNeaten.fire_wheel=ATTACK_V_PARAM.fire_wheel_status ;

		CanCommunit_t.gmTOch .dataNeaten .shoot =ATTACK_V_PARAM.COUNT ;
		CanCommunit_t.gmTOch .dataNeaten.vision =RUI_V_CONTAL.MOD[0];
//		CanCommunit_t.gmTOch.dataNeaten.chMod = chassisMod;
//		CanCommunit_t.gmTOch.dataNeaten.romoteOnLine = remoteOnLine;
//		CanCommunit_t.gmTOch.dataNeaten.topSate = topSate;
		//CanCommunit_t.gmTOch.dataNeaten.ptichAgnle = (int16_t)(-QEKF_INS.Roll*100);	//陀螺仪角度，单位0.1°

//						CanCommunit_t.gmTOch.dataNeaten.target=VISION_V_DATA.RECEIVE  .TARGET ;

//		IMU_Data.angle.pitch = INS.Pitch;
//		IMU_Data.angle.yaw = INS.YawTotalAngle;
		
		return 1;
}
/**
  * @author: 楠
  * @performance: CAN发送调用函数（云台to底盘）
  * @parameter: CAN1 or CAN2
  * @time: 23-7-9			
  * @ReadMe: 
*/

uint8_t CANGimbalTX( VT13_Typedef *VT13_DBUS)
{	
	//先将要发送的数据解算出来	
	GimbalTXResovle(VT13_DBUS);
	//数据发送部分
		canx_send_data(&hcan1, GIMBAL_kong, CanCommunit_t.gmTOch.sendData);		//数据发送
	
//	if(ppp%2 == 1)
//	{
//		canx_send_data(&can1, df_GMIMU_ID, IMU_Data.Data,8);
//	}
	
	

	return 1;
}


float testing_q0;
float testing_q1;
float testing_q2;
float testing_q3;

	float trans_q1;
	float trans_q2;
	float trans_q3;


void Quaternion_testing(IMU_Data_t *IMU)
{
trans_q1= IMU->yaw *0.01745329222222/2;
trans_q2= IMU->pitch*0.01745329222222/2;
trans_q3=IMU->roll*0.01745329222222/2;
  float sinhp;
	float sinhy;
	float sinhr;
	float coshp;
	float coshy;
	float coshr;
	float www;
	float xxx;
	float yyy;
	float zzz;

	sinhy=sin(trans_q1);  coshy=cos(trans_q1);
	sinhp=sin(trans_q2);  coshp=cos(trans_q2);
	sinhr=sin(trans_q3);  coshr=cos(trans_q3);
	www=coshp*coshy*coshr+sinhp*sinhy*sinhr;
	xxx=sinhr*coshp*coshy-coshr*sinhp*sinhy;
	yyy=coshy*sinhp*coshr+sinhy*coshp*sinhr;
	zzz=sinhy*coshp*coshr-coshy*sinhp*sinhr;
	testing_q0=www/(sqrt(www*www+xxx*xxx+yyy*yyy+zzz*zzz));
	testing_q1=xxx/(sqrt(www*www+xxx*xxx+yyy*yyy+zzz*zzz));
	testing_q2=yyy/(sqrt(www*www+xxx*xxx+yyy*yyy+zzz*zzz));
	testing_q3=zzz/(sqrt(www*www+xxx*xxx+yyy*yyy+zzz*zzz));
}
///计算俯仰角的弧度制
float cos_caculate(IMU_Data_t *IMU)
{
	return (IMU->pitch *3.1415926535 /180);
}
//pitch补偿
float addvice;
float adding_current;
float xiang[8];
float pitch_caculate(IMU_Data_t *IMU)
{
	addvice=cos_caculate(&IMU_Data);
	xiang[0]=2356.0 * sin((3.7188 *addvice) - 2.4654);
	xiang[1]=10905 * sin((7.8522 *addvice) - 0.0634);
	xiang[2]=9561.2 * sin((8.2273* addvice) - 3.2549);
	xiang[3]=1650.2 * sin((96.7152 *addvice) - 2.0901);
	xiang[4]=1687.8 * sin((96.8208 *addvice) + 1.0323);
	xiang[5]=-45.7669 * sin((77.7559 *addvice) + 5.4568);
	xiang[6]=49.6414 * sin((139.9559 *addvice) + 2.1662);
	xiang[7]=45.8386 * sin((181.8991 *addvice) + 2.5451);
	int i;
	for(i=0;i<8;i++)
	{
		adding_current+=xiang[i];
	}
	return adding_current;
}
void testinginginging()
{
	
}

void Encodeing_control(MOTOR_Typdef *MOTOR,DBUS_Typedef *WHW_V_DBUS)//编码器控制云台电机
{
	MOTOR->m_dm4310_p_t .DATA .Aim +=WHW_V_DBUS->Remote .CH2_int16 *0.01;
	    PID_Calculate(&MOTOR->m_dm4310_p_t .PID_P,
                  MOTOR->m_dm4310_p_t .DATA .reality   ,
                  MOTOR->m_dm4310_p_t .DATA.Aim);
    PID_Calculate(&MOTOR->m_dm4310_p_t .PID_S,
                 MOTOR->m_dm4310_p_t .DATA .Speed_now  ,
                  MOTOR->m_dm4310_p_t.PID_P.Output);
 dm4310_current_set(&hcan1,0x4FE,0 ,0,0,0);

}

////自己的编码器双环与陀螺仪双环
void pid_encoding_yawcontral(PID_t *pid,float target,float reality)
{
	pid->Err =target-reality;
	pid->Pout =pid->Kp *pid->Err ;
	pid->Iout +=pid->Ki *pid->Err ;
	pid->Dout =pid->Kd *(pid->Err -pid->Last_Err );
	pid->Output =pid->Pout +pid->Iout +pid->Dout ;
	pid->Last_Err =pid->Err ;
	
}
void pid_IMU_contal(MOTOR_Typdef *MOTOR,IMU_Data_t *IMU,float target,float reality)
{
}
float kkk;
float qqq;
float rrr;
float ooo;
float COS_pitch()
{
	ooo= (5500*cos(0.017453292519*(2.5*IMU_Data.pitch -5)));
	return ooo;
}





//		//键位赋值
//		CanCommunit_t.gmTOch.dataNeaten.key_v = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_V;
//		CanCommunit_t.gmTOch.dataNeaten.key_q = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_Q;
//		CanCommunit_t.gmTOch.dataNeaten.key_e = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_E;
//	    CanCommunit_t.gmTOch.dataNeaten.key_g=DBUS->KeyBoard.G_PreeNumber;
//	    CanCommunit_t.gmTOch.dataNeaten.key_x=DBUS->KeyBoard .X_PreeNumber ;
//		CanCommunit_t.gmTOch.dataNeaten.key_f=DBUS->KeyBoard .X ; //不规范写法 后边改过来
//		CanCommunit_t.gmTOch.dataNeaten.key_c = DBUS->KeyBoard .C_PreeNumber ;
//		CanCommunit_t.gmTOch.dataNeaten.key_r = DBUS->KeyBoard .R_PreeNumber ;
//		CanCommunit_t.gmTOch.dataNeaten.key_ctrl = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_Ctrl ;
//		CanCommunit_t.gmTOch.dataNeaten.supUSe = DBUS->KeyBoard.C ;
//		CanCommunit_t .gmTOch .dataNeaten .DBUS_state=RUI_ROOT_STATUS.RM_DBUS ;
////		CanCommunit_t.gmTOch.dataNeaten.chMod = chassisMod;
////		CanCommunit_t.gmTOch.dataNeaten.romoteOnLine = remoteOnLine;
////		CanCommunit_t.gmTOch.dataNeaten.topSate = topSate;
//		//CanCommunit_t.gmTOch.dataNeaten.ptichAgnle = (int16_t)(-QEKF_INS.Roll*100);	//陀螺仪角度，单位0.1°
//		CanCommunit_t.gmTOch .dataNeaten .S1 =DBUS->Remote .S1_u8 ;
//		CanCommunit_t.gmTOch .dataNeaten .S2 =DBUS->Remote .S2_u8 ;
//		CanCommunit_t.gmTOch.dataNeaten.target=VISION_V_DATA.RECEIVE .TARGET ;
////		IMU_Data.angle.pitch = INS.Pitch;
////		IMU_Data.angle.yaw = INS.YawTotalAngle;

