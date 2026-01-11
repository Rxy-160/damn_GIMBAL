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
//
	
    //云台电机初始化
    float PID_F_Pitch[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_P_Pitch[3] = {   -3.4f,   0.0f,   0.0f   };//{   -3.0f,   0.0f,   0.0f   };
    float PID_S_Pitch[3] = {   /*150.0f*/36,   0.0f,   0.0f   };//{   /*150.0f*/25,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_6020_Pitch.PID_F, 3000, PID_F_Pitch,
//                     0.5f, 2, 2);
		
    PID_Init(&MOTOR->m_dm4310_p_t .PID_P, 30000.0f, 100.0f,
             PID_P_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->m_dm4310_p_t .PID_S, 20000.0f, 100.0f,
             PID_S_Pitch, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             );//微分先行,微分滤波器

    float PID_F_Yaw[3] = {   0.0f,   0.0f,   0.0f   };
    float PID_P_Yaw[3] = {   3.0f,   0.0f,   0.0f   };//{   3.0f,   0.0f,   0.0f   };
    float PID_S_Yaw[3] = {   /*160.0f*/24,   0.0f,   0.0f   };//{   /*160.0f*/24,   0.0f,   0.5f   };
//    Feedforward_Init(&MOTOR->DJI_6020_Yaw.PID_F, 3000, PID_F_Yaw,
//                     0.5f, 2, 2);
//		
    PID_Init(&MOTOR->m_dm4310_y_t  .PID_P , 30000.0f, 0.0f,
             PID_P_Yaw, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->m_dm4310_y_t .PID_S, 30000.0f, 0.0f,
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
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Yaw  , 1);
    RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_6020_Pitch , 1);

    return RUI_DF_READY;
}
int a=0;
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
//    
		
///底盘跟随记得改回来
		/////VOFA调用
		VOFA_justfloat(VISION_V_DATA.RECEIVE .PIT_DATA ,
		               IMU->pitch  ,
		               VISION_V_DATA.RECEIVE .YAW_DATA  ,
		               -IMU->YawTotalAngle,
		               VISION_V_DATA.RECEIVE .TARGET  ,
		               IMU->gyro  [2] ,0,0,0,0);
//				VOFA_justfloat(IMU->gyro_correct [0] ,
//		               IMU->gyro_correct [1]  ,
//		               IMU->gyro_correct [2]  ,
//		               IMU->YawTotalAngle,
//		               0 ,
//		               0,0,0,0,0);

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

    /*目标值赋值*/

    MOTOR->m_dm4310_p_t .DATA.Aim = CONTAL->HEAD.Pitch;
    MOTOR->m_dm4310_y_t .DATA.Aim = CONTAL->HEAD.Yaw;

//    if(/*CONTAL->MOD[0] - CONTAL->MOD[1] == 1*/CONTAL->MOD [0]==0)//自瞄模式/////手瞄到自瞄
//    {
//        float PID_P_Yaw_vision[3] = {   -3.0f,   0.0f,   0.0f   };
//        float PID_S_Yaw_vision[3] = {   20.0f,   0.0f,   0.0f   };
//				float PID_P_Pitch_vision[3] = {   2.0f,   0.0f,   0.0f   };
//        float PID_S_Pitch_vision[3] = {   20.0f,   0.0f,   0.0f   };
//	//////////这个函数的目的是把不同模式的pid给赋进去
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_P, PID_P_Yaw_vision);
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_S, PID_S_Yaw_vision);
//				PID_set(&MOTOR->m_dm4310_p_t .PID_P, PID_P_Pitch_vision);
//        PID_set(&MOTOR->m_dm4310_p_t .PID_S, PID_S_Pitch_vision);
//    }
//    else if(/*CONTAL->MOD[0] - CONTAL->MOD[1] <= 0*/CONTAL->MOD [0]==1)//手瞄模式///////自瞄到手瞄
//    {
//        float PID_P_Yaw_hand[3] = {   -4.8f,   0.0f,   0.0f   };
//        float PID_S_Yaw_hand[3] = {   32.0f,   0.0f,   0.0f   };
//				float PID_P_Pitch_hand[3] = {   3.0f,   0.0f,   0.0f   };
//        float PID_S_Pitch_hand[3] = {  24.0f,   0.0f,   0.0f   };
//	//////////这个函数的目的是把不同模式的pid给赋进去
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_P, PID_P_Yaw_hand);
//        PID_set(&MOTOR->m_dm4310_y_t  .PID_S, PID_S_Yaw_hand);
//				PID_set(&MOTOR->m_dm4310_p_t .PID_P, PID_P_Pitch_hand);
//        PID_set(&MOTOR->m_dm4310_p_t .PID_S, PID_S_Pitch_hand);
//    }
//    CONTAL->MOD[1] = CONTAL->MOD[0];


    /*遥控离线保护*/
    if(!Root->RM_DBUS)
    {
			  MOTOR->m_dm4310_p_t  .PID_P .IntegralLimit =0;
			  MOTOR->m_dm4310_p_t  .PID_S .IntegralLimit =0;
        MOTOR->m_dm4310_p_t .PID_P.IntegralLimit = 0;
        MOTOR->m_dm4310_p_t .PID_S.IntegralLimit = 0;
        MOTOR->m_dm4310_p_t .DATA.Aim = (float)IMU->pitch *22.7555556f;
			
			  MOTOR->m_dm4310_y_t  .PID_P .IntegralLimit =0;
			  MOTOR->m_dm4310_y_t  .PID_S .IntegralLimit =0;
        MOTOR->m_dm4310_y_t .PID_P.IntegralLimit = 0;
        MOTOR->m_dm4310_y_t .PID_S.IntegralLimit = 0;
        MOTOR->m_dm4310_y_t .DATA.Aim = (float)IMU->YawTotalAngle *22.7555556f;

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
    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_P,
                  IMU->YawTotalAngle * 22.75555555555556f,
                  MOTOR->m_dm4310_y_t .DATA.Aim);
    PID_Calculate(&MOTOR->m_dm4310_y_t .PID_S,
                  IMU->gyro[2] * 100.0f,
                  MOTOR->m_dm4310_y_t .PID_P.Output);

    /*总输出计算*/

    tmp_G[0] = /*MOTOR->DJI_6020_Pitch.PID_F.Output */ 0 +
               MOTOR->m_dm4310_y_t  .PID_S.Output;

    tmp_G[1] = /*MOTOR->DJI_6020_Yaw.PID_F.Output*/0 +
               MOTOR->m_dm4310_p_t .PID_S.Output;
    /*CAN发送*/
//    DJI_Current_Ctrl(&hcan1,
//                     0x1FF,
//                     (int16_t)tmp_G[0],
//                     (int16_t)tmp_G[0],
//                     0,
//                     0);
++a;
//if(a%2==0)
//{
//	dm4310_current_set(&hcan1,0x3FE,tmp_G[0],0,0,0);

//  osdelay(1);
//}
//if(a%2==1)
//{
	dm4310_current_set(&hcan1,0x3FE,tmp_G[0],tmp_G[1]-3400/*-cos_caculate(IMU_Data)*/,0,0);
//}
//	dm4310_current_set(&hcan1,0x4FE,0,/*cos_caculate(IMU_Data)*/0,0,tmp_G[1]);

//dm4310_current_set(&hcan1,0x3FE,tmp_G[0],tmp_G[1],0,0);

//pos_ctrl(&hcan1,0x03,MOTOR->m_dm4310_p_t .DATA.Aim ,2);

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
		
//		if(CANID == df_CHControlAngle_ID && vision_boll == 1){	//处理角度信息
//				visionData_t.receive.pitchAngle[df_now] = ((float)CanCommunit_t.chTOgm.dataNeaten_angle.pitch / 100.0f);
//				visionData_t.receive.yawAngle[df_now] = ((float)CanCommunit_t.chTOgm.dataNeaten_angle.yaw / 100.0f);
//				RunTime = (uint64_t)(CanCommunit_t.chTOgm.dataNeaten_angle.time);
//		}
//		else if(CANID == df_CHControlData_ID){	//其余数据处理
//				//对裁判系统的处理
//				if(CanCommunit_t.chTOgm.dataNeaten_another.judgeState){
//						attack_t.muzzle.ammoNumber =CanCommunit_t.chTOgm.dataNeaten_another.muzzleColing / 100;//剩余弹量
//						twoJudgeData.muzzle.maxShootSpeed = CanCommunit_t.chTOgm.dataNeaten_another.maxSpeed;		//最大射速
//						
//						twoJudgeData.muzzle.shootSpeed = CanCommunit_t.chTOgm.dataNeaten_another.nowSpeed;		//当前射速
//						if( (attack_t.muzzle.adjustLock & 0x01) == 1){		//检测是否是自己上的锁
//								attack_t.muzzle.adjustLock = df_unClock;		//射速检测开锁
//						}
//						root_t.judgeRoot.time = 0;
//				}
//				//对视觉的处理
//				if(CanCommunit_t.chTOgm.dataNeaten_another.visionState){
//						visionData_t.receive.target = CanCommunit_t.chTOgm.dataNeaten_another.target;		//识别状态
//						visionData_t.receive.visionState = CanCommunit_t.chTOgm.dataNeaten_another.visionMod;	//视觉的模式
//						vision_boll = 1;
//						root_t.visionRoot.time = 0;
//				}
//				else{
//						vision_boll = 0;
//				}
////				if(CanCommunit_t.chTOgm.dataNeaten_another.judgeState){
////						
////				}
//		}
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
uint8_t GimbalTXResovle(  DBUS_Typedef *DBUS) 
{
	
//		int16_t pitchAngle = (int16_t)SectionLimit_f(500.0f, -500.0f, (TopData_t.pitchAgnle_f * 10.0f) );
		
//		KeyboardResolve();		//键盘模式底盘速度的解算
	
		CanCommunit_t.gmTOch.dataNeaten.vx =  DBUS->Remote .CH0_int16 ;
		CanCommunit_t.gmTOch.dataNeaten.vx += gimbal_t.Keyboard.vx;//键鼠，还没加
		CanCommunit_t.gmTOch.dataNeaten.vy =  DBUS->Remote .CH1_int16 ;
		CanCommunit_t.gmTOch.dataNeaten.vy += gimbal_t.Keyboard.vy;//
		CanCommunit_t.gmTOch.dataNeaten.vr =  -DBUS->Remote .Dial_int16 ;
		CanCommunit_t.gmTOch.dataNeaten.vr += gimbal_t.Keyboard.vr;//
		
		//狙击模式底盘上锁
		if(DBUS->KeyBoard .G_PreeNumber ==1 || DBUS->KeyBoard.B_PreeNumber ==1)
		{	
			CanCommunit_t.gmTOch.dataNeaten.vx=0;
			CanCommunit_t.gmTOch.dataNeaten.vy=0;
			CanCommunit_t.gmTOch.dataNeaten.vr=0;
//			chassisMod = 1;
		}
		
		//掉头模式底盘坐标系反转
		if(DBUS->KeyBoard .X_PreeNumber==1)
		{
			CanCommunit_t.gmTOch.dataNeaten.vx *= -1;
			CanCommunit_t.gmTOch.dataNeaten.vy *= -1;
		}
		
		//键位赋值
		CanCommunit_t.gmTOch.dataNeaten.key_v = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_V;
		CanCommunit_t.gmTOch.dataNeaten.key_q = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_Q;
		CanCommunit_t.gmTOch.dataNeaten.key_e = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_E;
	    CanCommunit_t.gmTOch.dataNeaten.key_g=DBUS->KeyBoard.G_PreeNumber;
	    CanCommunit_t.gmTOch.dataNeaten.key_x=DBUS->KeyBoard .X_PreeNumber ;
		CanCommunit_t.gmTOch.dataNeaten.key_f=DBUS->KeyBoard .X ; //不规范写法 后边改过来
		CanCommunit_t.gmTOch.dataNeaten.key_c = DBUS->KeyBoard .C_PreeNumber ;
		CanCommunit_t.gmTOch.dataNeaten.key_r = DBUS->KeyBoard .R_PreeNumber ;
		CanCommunit_t.gmTOch.dataNeaten.key_ctrl = RUI_V_DBUS_UNION.DataNeaten .KeyBoard_Ctrl ;
		CanCommunit_t.gmTOch.dataNeaten.supUSe = DBUS->KeyBoard.C ;
		CanCommunit_t .gmTOch .dataNeaten .DBUS_state=RUI_ROOT_STATUS.RM_DBUS ;
//		CanCommunit_t.gmTOch.dataNeaten.chMod = chassisMod;
//		CanCommunit_t.gmTOch.dataNeaten.romoteOnLine = remoteOnLine;
//		CanCommunit_t.gmTOch.dataNeaten.topSate = topSate;
		//CanCommunit_t.gmTOch.dataNeaten.ptichAgnle = (int16_t)(-QEKF_INS.Roll*100);	//陀螺仪角度，单位0.1°
		CanCommunit_t.gmTOch .dataNeaten .S1 =DBUS->Remote .S1_u8 ;
		CanCommunit_t.gmTOch .dataNeaten .S2 =DBUS->Remote .S2_u8 ;
		CanCommunit_t.gmTOch.dataNeaten.target=VISION_V_DATA.RECEIVE .TARGET ;
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

uint8_t CANGimbalTX(DBUS_Typedef *WHW_V_DBUS)
{	
	//先将要发送的数据解算出来	
	GimbalTXResovle(WHW_V_DBUS);
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
