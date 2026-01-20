#include "Shoot_Task.h"
TYPEDEF_ATTACK_PARAM ATTACK_V_PARAM = {0};

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
    float PID_F_L[3] = {   2.0f,   0.0f,   0.0f   };
    float PID_S_L[3] = {   2.0f,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_3508_Shoot_L.PID_F, 3000, PID_F_L,
//                     0.5f, 2, 2);
//    PID_Init(&MOTOR->DJI_3508_Shoot_L.PID_S, 6000.0f, 2000.0f,
//             PID_S_L, 1000.0f, 1000.0f,
//             0.7f, 0.7f, 2,
//             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
		    PID_Init(&MOTOR->DJI_3508_Shoot_L.PID_S , 30000.0f, 2000.0f,
             PID_S_L, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    float PID_F_R[3] = {   2.3f,   0.0f,   0.0f   };
    float PID_S_R[3] = {   2.3f,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_3508_Shoot_R.PID_F, 3000, PID_F_R,
//                     0.5f, 2, 2);
//    PID_Init(&MOTOR->DJI_3508_Shoot_R.PID_S, 6000.0f, 2000.0f,
//             PID_S_R, 1000.0f, 1000.0f,
//             0.7f, 0.7f, 2,
//             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
		    PID_Init(&MOTOR->DJI_3508_Shoot_R.PID_S , 30000.0f, 2000.0f,
             PID_S_R, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

    float PID_F_M[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_P_M[3] = {   0.2f,   0.0f,   0.0f   };
    float PID_S_M[3] = {   2.0f,   0.0f,   0.0f   };
//    Feedforward_Init(&MOTOR->DJI_3508_Shoot_M.PID_F, 3000, PID_F_M,
//                     0.5f, 2, 2);
//    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_P, 6000.0f, 2000.0f,
//             PID_P_M, 1000.0f, 1000.0f,
//             0.7f, 0.7f, 2,
//             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器
		    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_P , 30000.0f, 2000.0f,
             PID_P_M, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
		    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_S , 30000.0f, 2000.0f,
             PID_S_M, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器

//    PID_Init(&MOTOR->DJI_3508_Shoot_M.PID_S, 6000.0f, 2000.0f,
//             PID_S_M, 1000.0f, 1000.0f,
//             0.7f, 0.7f, 2,
//             Integral_Limit|OutputFilter|ErrorHandle//积分限幅,输出滤波,堵转监测
//             |Trapezoid_Intergral|ChangingIntegrationRate//梯形积分,变速积分
//             |Derivative_On_Measurement|DerivativeFilter);//微分先行,微分滤波器

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
	////不能注释
    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Shoot_INIT(MOTOR);
        return RUI_DF_ERROR;
    }
/////必须注释
//    /*电机在线检测*/
//    if (AIM_INIT != RUI_DF_READY)
//    {
//        AIM_INIT = Shoot_AIM_INIT(Root, MOTOR);
//        return RUI_DF_ERROR;
//    }

    /*目标值赋值*/
//    MOTOR->DJI_3508_Shoot_L.DATA.Aim = CONTAL->SHOOT.SHOOT_L;
//    MOTOR->DJI_3508_Shoot_R.DATA.Aim = CONTAL->SHOOT.SHOOT_R;
//    MOTOR->DJI_3508_Shoot_M.DATA.Aim = CONTAL->SHOOT.SHOOT_M;

    /*遥控离线保护*/
    if(!Root->RM_DBUS)
    {
//        MOTOR->DJI_3508_Shoot_L.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_L.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_R.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_R.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_M.PID_P.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_M.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_M.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_M .DATA.Angle_now ;

//			PID_INIT = RUI_DF_ERROR;
			AIM_INIT = RUI_DF_ERROR;
    }

    /*堵转处理*/
//    if(MOTOR->DJI_3508_Shoot_L.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_L, 0);
//    if(MOTOR->DJI_3508_Shoot_R.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_R, 0);
//    if(MOTOR->DJI_3508_Shoot_M.PID_P.ERRORHandler.ERRORType & Motor_Blocked)
//        RUI_F_HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Shoot_M, 0);

//    /*Shoot_L*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_F,
//                          MOTOR->DJI_3508_Shoot_L.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_L.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now,
                  MOTOR->DJI_3508_Shoot_L.DATA.Aim);

//    /*Shoot_R*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_F,
//                          MOTOR->DJI_3508_Shoot_R.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_R.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now,
                  MOTOR->DJI_3508_Shoot_R.DATA.Aim);

//    /*Shoot_M*/
//    Feedforward_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_F,
//                          MOTOR->DJI_3508_Shoot_M.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_P,
                  (float)MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite ,
                  MOTOR->DJI_3508_Shoot_M.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Shoot_M.PID_S,
                  (float)MOTOR->DJI_3508_Shoot_M.DATA.Speed_now,
                  MOTOR->DJI_3508_Shoot_M.PID_P.Output);

    /*总输出计算*/
    float tmp_S[3];

    tmp_S[0] = MOTOR->DJI_3508_Shoot_L.PID_F.Output +
               MOTOR->DJI_3508_Shoot_L.PID_S.Output;

    tmp_S[1] = MOTOR->DJI_3508_Shoot_R.PID_F.Output +
               MOTOR->DJI_3508_Shoot_R.PID_S.Output;

    tmp_S[2] = MOTOR->DJI_3508_Shoot_M.PID_F.Output +
               MOTOR->DJI_3508_Shoot_M.PID_S.Output;

    /*CAN发送*/
    DJI_Current_Ctrl(&hcan2,
                     0x200,
                     (int16_t)tmp_S[0],
                     (int16_t)tmp_S[1],
                     (int16_t)tmp_S[2],
                     0);

//    /*发射机构信息反馈*/
//    CONTAL->SHOOT_Bask.Speed_Aim_L = MOTOR->DJI_3508_Shoot_L.DATA.Aim;
//    CONTAL->SHOOT_Bask.Speed_Aim_R = MOTOR->DJI_3508_Shoot_R.DATA.Aim;
//    CONTAL->SHOOT_Bask.Speed_err_L = MOTOR->DJI_3508_Shoot_L.DATA.Aim -
//                                     (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now;
//    CONTAL->SHOOT_Bask.Speed_err_R = MOTOR->DJI_3508_Shoot_R.DATA.Aim -
//                                     (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now;
//    CONTAL->SHOOT_Bask.Angle = MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite;
//    CONTAL->SHOOT_Bask.JAM_Flag = RUI_F_JAM(&MOTOR->DJI_3508_Shoot_M.DATA, CONTAL);
//    CONTAL->SHOOT_Bask.Shoot_Number = MOTOR->DJI_3508_Shoot_M.DATA.Angle_Infinite / CONTAL->SHOOT.Single_Angle;

    return RUI_DF_READY;
}


// @TODO 整合到整个ROOT_init函数中, 记得Init 时间 && 摩擦轮目标值应根据裁判系统拟合
void ATTACK_F_Init(MOTOR_Typdef *MOTOR)
{
	  static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;
	////不能注释
    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = MOTOR_PID_Shoot_INIT(MOTOR);
    }

	//离线检测
	    if(!RUI_ROOT_STATUS.RM_DBUS)
    {
//        MOTOR->DJI_3508_Shoot_L.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_L.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_L.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_R.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_R.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_R.DATA.Speed_now;

//        MOTOR->DJI_3508_Shoot_M.PID_P.IntegralLimit = 0;
//        MOTOR->DJI_3508_Shoot_M.PID_S.IntegralLimit = 0;
        MOTOR->DJI_3508_Shoot_M.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_M .DATA.Angle_now ;

//			PID_INIT = RUI_DF_ERROR;
			AIM_INIT = RUI_DF_ERROR;
    }

//    ATTACK_V_PARAM.TIME = 0;

    // 数据初始化
    ATTACK_V_PARAM.SINGLE_ANGLE = 81920;//8192/9*36*50/20单发角度
    ATTACK_V_PARAM.SPEED = 7000.0f;//摩擦轮速度

    ATTACK_V_PARAM.FLAG = 1;
    ATTACK_V_PARAM.LOCK = 1; // 默认上锁，保证在未收到遥控数据时拨盘不动

    // 电机初始化
    MOTOR->DJI_3508_Shoot_L.DATA.Aim=  0.0f;//////左右摩擦轮
    MOTOR->DJI_3508_Shoot_R.DATA.Aim =  0.0f;
    MOTOR->DJI_3508_Shoot_M.DATA.Aim=  (float)MOTOR->DJI_3508_Shoot_M  .DATA .Angle_Infinite ;///连续编码

}

/**
 * @brief 根据遥控/鼠标输入计算拨弹电机目标值
 * @param MOTOR 拨弹电机指针
 * @param DBUS 遥控器数据指
 * @param autofire 视觉控制开火
 * @return 目标角度值
 */
void ATTACK_F_JAM_Aim(MOTOR_Typdef *MOTOR, DBUS_Typedef *DBUS, uint8_t autofire)
{
    // 记录上一次的遥控器和鼠标状态
    static uint8_t prev_S2_state = DBUS_D_MOD_SHUT;
    
    // 单发模式处理 (通过状态变化检测)
    if (DBUS->Remote .S2_u8 == DBUS_D_MOD_SINGLE)//这一次是单发
    {
        // 检测遥控器模式切换到单发     2
        if (prev_S2_state != DBUS_D_MOD_SINGLE)//上一次不是单发
        {
					//////是否进入视觉模式
//            if (autofire == 0 || ((autofire == 1) && VISION_V_DATA.RECEIVE.fire))
//            {
                ATTACK_V_PARAM.COUNT = 1;  // 增加一个弹丸的角度
//            }
        }
    }
//    // 鼠标单发处理
//		///键鼠
//    else if (DBUS->MOUSE.L_STATE == 1 && ATTACK_V_PARAM.PREV_MOUSE_STATE != 1)
//    {
//        if (autofire == 0 || ((autofire == 1) && VISION_V_DATA.RECEIVE.fire))
//        {
//            ATTACK_V_PARAM.COUNT = 1;  // 检测到鼠标左键按下事件，发射一个弹丸
//        }
// 		}
		
///连发模式记得解锁
		
    // 连发模式处理                            1
    else if (DBUS->Remote .S2_u8 == DBUS_D_MOD_CONSIST /*||VISION_V_DATA.RECEIVE .fire ==1*//*|| DBUS->MOUSE.L_STATE == 2*/)
    {
//        if (/*autofire == 0 || ((autofire == 1) && VISION_V_DATA.RECEIVE.fire)*/VISION_V_DATA.RECEIVE.TARGET ==1)
//        {
            // 视觉允许开火且为连发模式
            ATTACK_V_PARAM.COUNT = 1;  // 持续小量增加目标角度，形成连续转动
//        }
    }
    // 关闭发射处理
    else if (DBUS->Remote.S2_u8 == DBUS_D_MOD_SHUT)
    {
        ATTACK_V_PARAM.COUNT = 0;  // 关闭状态不增加角度
    }

    // 计算新的电机目标角度                              摩擦轮开启
    if (ATTACK_V_PARAM.COUNT > 0 && ATTACK_V_PARAM.fire_wheel_status /*&& MOTOR->DATA.ENABLE*/) // @debug  
    {
			MOTOR->DJI_3508_Shoot_M.DATA.Aim =MOTOR->DJI_3508_Shoot_M .DATA .Angle_Infinite-/*36864*/20000;
//        MOTOR->DJI_3508_Shoot_M.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_M .DATA .Angle_Infinite - (ATTACK_V_PARAM.SINGLE_ANGLE * ATTACK_V_PARAM.COUNT);
		// 单发模式下，处理完一次后重置COUNT
        if (DBUS->Remote .S2_u8 == DBUS_D_MOD_SINGLE/*||VISION_V_DATA.RECEIVE .fire ==1*//* || DBUS->MOUSE.L_STATE == 1*/)
        {
					  MOTOR->DJI_3508_Shoot_M.DATA.Aim = (float)MOTOR->DJI_3508_Shoot_M .DATA .Angle_Infinite - (ATTACK_V_PARAM.SINGLE_ANGLE * ATTACK_V_PARAM.COUNT);
            ATTACK_V_PARAM.COUNT = 0;
        }
    }
   
    // 保存当前状态用于下一次比较
//    ATTACK_V_PARAM.PREV_MOUSE_STATE = DBUS->MOUSE.L_STATE;
    prev_S2_state = DBUS->Remote .S2_u8 ;

}
// 获取摩擦轮目标值  @TODO 应该是一个函数，等会写一个如何获取拟合函数
// @TODO 电机当前转速也值得加入，但这之后再说
void ATTACK_F_FIRE_Aim(MOTOR_Typdef *MOTOR,DBUS_Typedef*DBUS)
{
    static uint8_t fire_mouse_status = 0;
    // @veision 1 拟合
    // float a = 0.0f, b = 0.0f;
    // ATTACK_V_PARAM.SPEED = user_data.shoot_data.initial_speed * a + b;
    // return ATTACK_V_PARAM.SPEED;

    // @version 2, Use this code, this code is get the fire speed by judgement system
    // but the weakness is that the speed is not stable
    
//    static float TEMP = 0.0f;
//    initial_speed[NOW] = user_data.shoot_data.initial_speed;
//    initial_speed[LAST] = initial_speed[NOW]; // 保证attack速度只在更新时修改摩擦轮转速
//    initial_speed[NOW] = user_data.shoot_data.initial_speed; // 更新摩擦轮转速
//		if (initial_speed[NOW] != initial_speed[LAST])
//		{
//				if(user_data.shoot_data.initial_speed <= 22.9f)
//				{
//						TEMP += 4;
//				}
//				else if (user_data.shoot_data.initial_speed > 22.9f && user_data.shoot_data.initial_speed <= 23.2f)
//				{
//						TEMP += 1.5f;
//				}
//				else if (user_data.shoot_data.initial_speed > 23.2f && user_data.shoot_data.initial_speed <= 23.5f)
//				{
//						TEMP += 0;
//				}
//				else if (user_data.shoot_data.initial_speed > 23.5f && user_data.shoot_data.initial_speed <= 23.9f)
//				{
//						TEMP -= 5;
//				}
//				else if (user_data.shoot_data.initial_speed > 23.9f)
//				{
//						TEMP -= 15;
//				}
//		}
		
    ATTACK_V_PARAM.SPEED = 7200.0f ;//+ TEMP;
//    ATTACK_V_PARAM.SPEED = MATH_D_LIMIT(6370.0f, 6180.0f, ATTACK_V_PARAM.SPEED);

    // @veision 3, final code, this code is a stable speed
		//键鼠：s2在中间   或     s2在2是开火状态
    if (/*( fire_mouse_status == 1 && DBUS->Remote .S1_u8 == 3)||*/ /*DBUS->Remote.S1_u8 ==3||*/DBUS->Remote.S1_u8 == 2)  // 3 is the fire button
    {
        MOTOR->DJI_3508_Shoot_L .DATA .Aim =  -ATTACK_V_PARAM.SPEED;
				MOTOR->DJI_3508_Shoot_R .DATA .Aim =  ATTACK_V_PARAM.SPEED;
        ATTACK_V_PARAM.fire_wheel_status = 1;
    } 
		else
    {
        MOTOR->DJI_3508_Shoot_L .DATA .Aim = 0;
				MOTOR->DJI_3508_Shoot_R .DATA .Aim = 0;
        ATTACK_V_PARAM.fire_wheel_status = 0;
    }
//    if (remote_t.KEY_BOARD.CTRL  && !DBUS_V_DATA.KEY_BOARD.CTRL_PREE_NUMBER)
//    {
//        fire_mouse_status = !fire_mouse_status;
//    }
//    DBUS_V_DATA.KEY_BOARD.CTRL_PREE_NUMBER = DBUS_V_DATA.KEY_BOARD.CTRL;
    
}


// 总控制函数
void ATTACK_F_Ctl(DBUS_Typedef *DBUS,MOTOR_Typdef *MOTOR)
{
//    if (!MOTOR[MOTOR_D_ATTACK_G].is_off[NOW] && MOTOR[MOTOR_D_ATTACK_G].is_off[LAST]) 
//		{
//        GM2006_demo//.DATA.AIM = MOTOR[MOTOR_D_ATTACK_G].DATA.ANGLE_INFINITE;
//    }
//    MOTOR[MOTOR_D_ATTACK_G].is_off[LAST] = MOTOR[MOTOR_D_ATTACK_G].is_off[NOW];

//    aim_get_edge = fabs((MOTOR[MOTOR_D_ATTACK_G].DATA.AIM) - (float)MOTOR[MOTOR_D_ATTACK_G].DATA.ANGLE_INFINITE);
//    if (aim_get_edge < 5000.0f) // @debug 5000.0f
//        ATTACK_V_PARAM.STATUS[NOW] = 0;
//    else ATTACK_V_PARAM.STATUS[NOW] = 1;
    
//    if (ATTACK_F_HeatControl(&MOTOR[MOTOR_D_ATTACK_G], 0) && ATTACK_V_PARAM.STATUS[1] == 0)    // 判断方向优先考虑热量
//    {
//      /*  GM2006_demo.DATA.AIM = */ATTACK_F_JAM_Aim(&MOTOR[MOTOR_D_ATTACK_G], DBUS, VISION_V_DATA.SEND.is_buff);
//    }
//    if (!ATTACK_F_JAM_Check(&MOTOR_V_ATTACK[MOTOR_D_ATTACK_G]))
//    {
//        // 卡弹
//        GM2006_demo.DATA.AIM = (float)GM2006_demo.DATA.ANGLE_INFINITE+ ATTACK_V_PARAM.SINGLE_ANGLE * ATTACK_V_PARAM.FLAG; //  @debug  * ATTACK_V_PARAM.FLAG
//        ATTACK_V_PARAM.FLAG = -ATTACK_V_PARAM.FLAG;
////        ATTACK_V_PARAM.TIME = 0; // 重置时间
//   // }
 //计算拨弹电机的目标值
   		ATTACK_F_JAM_Aim(&ALL_MOTOR ,&WHW_V_DBUS ,0);//最后一位没用
 // shooting case when opposite to you l-r
		//计算两个摩擦轮目标值
    ATTACK_F_FIRE_Aim(&ALL_MOTOR ,&WHW_V_DBUS );
//    ATTACK_F_JAM_Disable(&MOTOR[MOTOR_D_ATTACK_G]);/////卡弹处理函数
    // ATTACK_T_FIT(40);
		//对两个摩擦轮电机进行pid计算
    // pid
		PID_Calculate(&ALL_MOTOR.DJI_3508_Shoot_L.PID_S,MOTOR->DJI_3508_Shoot_L .DATA .Speed_now ,-(MOTOR->DJI_3508_Shoot_L .DATA .Aim) );
		PID_Calculate(&ALL_MOTOR.DJI_3508_Shoot_R.PID_S,MOTOR->DJI_3508_Shoot_R .DATA .Speed_now ,-(MOTOR->DJI_3508_Shoot_R .DATA .Aim) );
//    // PID_F_S(&MOTOR[MOTOR_D_ATTACK_L]);
//    // PID_F_S(&MOTOR[MOTOR_D_ATTACK_R]);
//		//对拨弹电机进行pid计算
		PID_Calculate(&ALL_MOTOR .DJI_3508_Shoot_M .PID_P ,MOTOR->DJI_3508_Shoot_M.DATA .Angle_Infinite  ,MOTOR->DJI_3508_Shoot_M .DATA .Aim );
		PID_Calculate(&ALL_MOTOR .DJI_3508_Shoot_M .PID_S ,MOTOR->DJI_3508_Shoot_M .DATA .Speed_now ,MOTOR->DJI_3508_Shoot_M .PID_P .Output);
		float tmp_S[3];

    tmp_S[0] = /*MOTOR->DJI_3508_Shoot_L.PID_F.Output +*/
               MOTOR->DJI_3508_Shoot_L.PID_S.Output;

    tmp_S[1] = /*MOTOR->DJI_3508_Shoot_R.PID_F.Output +*/
               MOTOR->DJI_3508_Shoot_R.PID_S.Output;

    tmp_S[2] =/*MOTOR->DJI_3508_Shoot_M.PID_F.Output +*/
               MOTOR->DJI_3508_Shoot_M.PID_S.Output;

    /*CAN发送*/
    DJI_Current_Ctrl(&hcan1,
                     0x200,
                     0,
                     0,
                     (/*int16_t*/float)tmp_S[2],
                     0);
	  DJI_Current_Ctrl(&hcan2,
                     0x200,
                     (float)tmp_S[0],
                     (float)tmp_S[1],
                     /*(int16_tfloat)tmp_S[2]*/0,
                     0);


}
//////////以下是火控部分
///**
// * @brief 弹频拟合函数
// * @param type 1:线性拟合 2:二次拟合 3:考虑卡弹 4:不考虑卡弹
// */
//float ATTACK_F_FireRate_Control(TYPEDEF_MOTOR *motor, float hz, uint8_t type)
//{
//    switch (type)
//    {
//    case 1: {// 线性拟合
//        // 线性拟合 (y = ax + b):      x = np.array([3000.0, 4500.0, 5000.0])
//        //                            y = np.array([13.3, 16.99, 23.3])
//        //                            方程: y = 0.0044x + -0.5277
//        //                            R² = 0.8253
//        float a = 0.0044f, b = -0.5277f;
//        float y = a * hz + b;
//        motor->PID_A.IN.ALL_LIT = y;
//        break;
//    }
//       
//    case 2: {// 二次拟合
//        float a = 0.0f, b = 0.0f, c = 0.0f;
//        float y = a * hz * hz + b * hz + c;
//        motor->PID_A.IN.ALL_LIT = y;
//        break;
//    }
//    case 3: { // 考虑退弹卡弹
//        const float TASK_RUN_TIME = 1.1f, JAM_COUNT = 10.0f; // 任务运行时间ms，JAM_COUNTs内卡弹1次数
//        float a = ATTACK_D_TIMEOUT * TASK_RUN_TIME / 1000.0f, P = 1.0f / (hz * JAM_COUNT); // 假设卡弹一次0.5s 卡弹概率为2s内一发
//        float fact_hz = 1.0f / ((1.0f - P) * 1.0f / hz + P * 2 * a);      // 考虑卡弹实际频率

//        compensation_hz = ATTACK_Calc_Hz_From_FactHz(hz, a, P);
//        compensation_hz = MATH_D_LIMIT(25.0f, 12.0f, compensation_hz); // 限制最大值
//        motor->PID_A.IN.ALL_LIT = compensation_hz * 60.0f * 4.5f;
//        break;
//    }
//    case 4: {
//        motor->PID_A.IN.ALL_LIT = hz * 60.0f * 4.5f;
//    }
//    default:
//        break;
//    }
//    return motor->PID_A.IN.ALL_LIT;
//}


///**
// * @brief 
// * 
// * @param motor 
// * @param type 类型选择 0停转 1减频
// * @return uint8_t 
// * @note type为0时，代码调用在获取拨弹目标值上，保证热量不足时不获取拨弹目标值
// * @note type为1时，代码直接调用，根据当前等级和热量控制弹频
// */
//uint8_t ATTACK_F_HeatControl(TYPEDEF_MOTOR *motor, uint8_t type) 
//{
//    float d = 10.0f, shoot_time = 0.0f, shoot_speed = 0.0f;           
//    float a = (float)(user_data.robot_status.shooter_barrel_cooling_value); // 冷却值 /s
//    float m = fabsf((float)(user_data.robot_status.shooter_barrel_heat_limit - user_data.power_heat_data.shooter_17mm_1_barrel_heat)); // 剩余可发热量 10*n
//    uint16_t leastbullet = (uint16_t)(m) / 10;
//    float rate = (m+a * 0.1f)/(float)user_data.robot_status.shooter_barrel_heat_limit;
//    if (a == 0) rate = 2.0f;  // 收不到裁判系统数据，设置为错误数据
//    
//    ATTACK_F_FireRate_Control(&aaa, 18.0f, 3);

//    // VOFA_T_Send(0, 10, (float)a, m, 
//    //                    (float)user_data.robot_status.shooter_barrel_heat_limit, 
//    //                    (float)user_data.power_heat_data.shooter_17mm_1_barrel_heat, 
//    //                    shoot_time, (float)leastbullet, (float)ATTACK_V_PARAM.SPEED, 
//    //                    (float)aaa.PID_S.IN.ALL_LIT, compensation_hz, 0); // @TODO 发送数据到VOFA
//    if (type == 0)
//    {
//        switch (user_data.robot_status.robot_level)
//        {
//            case 1:
//            {
//                if (leastbullet >=5) return 1;
//                else return 0;
//            }
//                break;
//            case 2:
//            {
//                if (leastbullet >= 5) return 1;
//                else return 0;
//            }
//                break;
//            case 3:case 4:
//            {
//                if (leastbullet >= 5) return 1;
//                else return 0;
//            }
//                break;
//            case 5:case 6:
//            {
//                if (leastbullet >= 5) return 1;
//                else return 0;
//            }
//                break;
//            case 7:
//            {
//                if (leastbullet >= 3) return 1;
//                else return 0;
//            }
//                break;
//            case 8:case 9:case 10:
//            {
//                if (leastbullet >= 4) return 1;
//                else return 0;
//            }
//                break;
//            default:
//            {
//                return 1;
//            }
//                break;
//            }
//        }
//    else if (type == 1)
//    {
//        static float fq = 15.0f;
//        if (rate <= 0) // 超热量
//        {
//            fq = 0.0f;
//        } else if (rate >0 && rate <= 1.0f) {
//            // rate 0->1 0->18
//            fq = -0.0586 * rate - 0.0431f;
//        } else {
//            fq = 15.0f; // 如果收不到裁判系统数据，定值
//        }
//        if (fq <= 0.0f) {
//            fq = 0.0f;
//        }
//        ATTACK_F_FireRate_Control(motor, fq, 3);
//    }
//    return 1;
//}


