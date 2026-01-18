#include "Robot.h"
float monitor_X;
float monitor_Y;
float monitor_W;
static float counttt=0.0f;
void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               TYPEDEF_VISION *Vision,
               RUI_ROOT_STATUS_Typedef *Root,
               MOTOR_Typdef *MOTOR,
               IMU_Data_t *IMU_Data)
{
    switch (mode) {

        case 1://底盘
        {
					////加速度
					MOTOR->DJI_3508_Chassis_1 .DATA .acceleration =MOTOR->DJI_3508_Chassis_1 .DATA .Speed_now -MOTOR->DJI_3508_Chassis_1 .DATA .Speed_last ;
					MOTOR->DJI_3508_Chassis_2 .DATA .acceleration =MOTOR->DJI_3508_Chassis_2 .DATA .Speed_now -MOTOR->DJI_3508_Chassis_2 .DATA .Speed_last ;
					MOTOR->DJI_3508_Chassis_3 .DATA .acceleration =MOTOR->DJI_3508_Chassis_3 .DATA .Speed_now -MOTOR->DJI_3508_Chassis_3 .DATA .Speed_last ;
					MOTOR->DJI_3508_Chassis_4 .DATA .acceleration =MOTOR->DJI_3508_Chassis_4 .DATA .Speed_now -MOTOR->DJI_3508_Chassis_4 .DATA .Speed_last ;
					//缓启动变量
            static float SLOW_START = 0.0f;
					            //缓制动变量
            static float SLOW_BLACK = 1.5f;

            //底盘跟随PID与前馈
            static float FIX_ANGLE = 0.0f;
            static float RUI_V_FOLLOW_PREDICT=0;
            //最大功率
            float MAX_POWER;

            if(DBUS->KeyBoard.V_PreeNumber && (CAP_DATA->GET.CAP_VOLT > 180.0f || Root->Power == RUI_DF_OFFLINE))
            {   //V键开电容,写死最大功率
                MAX_POWER = 250.0f;
            }
            else
            {   //正常模式,读取最大功率
                MAX_POWER = (User_data->robot_status.chassis_power_limit != 0) ? (float)User_data->robot_status.chassis_power_limit : 45.0f;
            }

            /*目标值传递*/
            CONTAL->BOTTOM.VY = (float) -(( DBUS->Remote.CH0_int16*6 ) + ( DBUS->KeyBoard.D - DBUS->KeyBoard.A ) * 660 );
            CONTAL->BOTTOM.VX = (float) -(( DBUS->Remote.CH1_int16*6 ) + ( DBUS->KeyBoard.W - DBUS->KeyBoard.S ) * 660 );
            CONTAL->BOTTOM.VW = (float) -(( DBUS->Remote.Dial_int16 *6) + DBUS->KeyBoard.Shift * 660 );
						
            /*缓启动*/
            if(CONTAL->BOTTOM.VX != 0 || CONTAL->BOTTOM.VY != 0 || CONTAL->BOTTOM.VW != 0)
            {
                SLOW_START += 0.0001f;//0.002f;
                float SLOW_START_MAX = RUI_F_CHASSIS_GET_MAX_TARGET(MAX_POWER);
                if(SLOW_START > SLOW_START_MAX)
                {
                    SLOW_START = SLOW_START_MAX;
                }
						CONTAL->BOTTOM.VX *= ( 1 - RUI_F_MATH_Limit_float(2750, 0, RUI_F_MATH_ABS_float(FIX_ANGLE)) / 2750.0f );
            CONTAL->BOTTOM.VY *= ( 1 - RUI_F_MATH_Limit_float(2750, 0, RUI_F_MATH_ABS_float(FIX_ANGLE)) / 2750.0f );


            CONTAL->BOTTOM.VX *= SLOW_START;
            CONTAL->BOTTOM.VY *= SLOW_START;
            CONTAL->BOTTOM.VW *= SLOW_START;
								
            monitor_X=CONTAL->BOTTOM.VX;
						monitor_Y=CONTAL->BOTTOM.VY;
						monitor_W=CONTAL->BOTTOM.VW;
            }

						

            
        } break;

        case 2://云台
        {
//            if(DBUS->Mouse.R_State & (int8_t) Vision->Target)//自瞄
//            {
//                

//                CONTAL->HEAD.Pitch = Vision->PitchAngle * 22.7555556f;

//                CONTAL->HEAD.Pitch = 100;//RUI_F_MATH_Limit_float(CONTAL->HEAD.Pitch_MAX,
//                                                           // CONTAL->HEAD.Pitch_MIN,
//                                                           // CONTAL->HEAD.Pitch);

//                CONTAL->HEAD.Yaw = Vision->YawAngle * 22.7555556f;
//            }
//            else//手瞄
//            {
               
                if(DBUS->Remote .S1_u8 ==1||DBUS->Remote .S1_u8 ==2)
								{
									CONTAL->MOD[0] = 1;//云台模式切换///////手瞄模式
                CONTAL->HEAD.Pitch -= (float) (DBUS->Remote.CH3_int16) * 0.004f +
                                      DBUS->Mouse.Y_Flt * 0.01f;

                CONTAL->HEAD.Pitch = RUI_F_MATH_Limit_float(CONTAL->HEAD.Pitch_MAX,
                                                            CONTAL->HEAD.Pitch_MIN,
                                                            CONTAL->HEAD.Pitch);

                CONTAL->HEAD.Yaw  -= (float) (DBUS->Remote.CH2_int16) * 0.006f +
                                    RUI_F_MATH_Limit_float(1, -1, DBUS->Mouse.X_Flt * 0.01f) +
                                    (float) (DBUS->KeyBoard.E - DBUS->KeyBoard.Q);
								}
								else if(DBUS->Remote .S1_u8 ==3)
								{
									CONTAL->MOD[0] = 0;//云台模式切换/////////////////////自瞄模式
									//改视觉模式极性
									if(Vision->RECEIVE .TARGET ==1)
									{		
									counttt++;
										if(counttt>100)
										{
											CONTAL->HEAD.Pitch =-5.1*22.755555555555 ;//-Vision->RECEIVE .PIT_DATA  *22.75555555555556f;
		//										RUI_F_MATH_Limit_float(CONTAL->HEAD.Pitch_MAX,
		//																						CONTAL->HEAD.Pitch_MIN,
		//																						Vision->RECEIVE .PIT_DATA  *22.75555555555556f);



											CONTAL->HEAD .Yaw   =  -Vision->RECEIVE  .YAW_DATA   *22.75555555555556f;
										}

									}
								else if(Vision->RECEIVE .TARGET ==0)
									{
											CONTAL->HEAD .Pitch =-5.1 *22.7555555555555;//IMU_Data->pitch *22.75555555555556f;
											CONTAL->HEAD .Yaw =IMU_Data ->YawTotalAngle*22.75555555555556f ;
									}
								}
//            }

        } break;

        case 3://电容(待修改)
        {

        } break;

        case 4://发射
        {
					
        } break;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-2-25 下午1:24
 *	@ReadMe:			//获取摩擦轮的目标值
 ************************************************************万能分隔符**************************************************************/
float Shoot_Speed_P(float Kp, float measure, float ref, float OUT_Lim)
{
    float error = ref - measure;
    /*比例输出*/
    float ALL_Out = error * Kp;
    /*总输出限幅*/
    ALL_Out = RUI_F_MATH_Limit_float(OUT_Lim, -OUT_Lim, ALL_Out);

    return ALL_Out;
}

float RUI_F_GET_FIRE_WIPE_SPEED(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, User_Data_T *User_data, RUI_ROOT_STATUS_Typedef *Root)
{
    static uint8_t LOCK = 0, MOD = 0, KEYBOARD_LOCK = 0, JUDGE_LOCK = 0;
    static float AIM = 0.0f, TEMP = 0.0f, SPEED_NOW = 0.0f, SPEED_LAST = 0.0f;
    MOD = DBUS->Remote.S2_u8-1 | DBUS->Mouse.L_State | LOCK;
    //停止
    if (MOD == 0 || DBUS->Remote.S2_u8-1 == 1 || Root->RM_DBUS == RUI_DF_OFFLINE)
    {
        AIM = 0.0f;//0.0
        LOCK = 0;
    } else
    {
        LOCK = 1;
        SPEED_LAST = SPEED_NOW;
        SPEED_NOW = User_data->shoot_data.initial_speed;

        /*弹速PID,仅限于修正摩擦轮发热所带来的弹速变化,优化为打一发弹算一次*/
        if (CONTAL->SHOOT_Bask.Shoot_Number - CONTAL->SHOOT_Bask.Shoot_Number_Last > 0)
        {
            if(RUI_F_MATH_ABS_float(SPEED_NOW - SPEED_LAST) > 0.1f)
                TEMP = Shoot_Speed_P(5.0f, SPEED_NOW, 29.5f, 50);
            CONTAL->SHOOT_Bask.Shoot_Number_Last = CONTAL->SHOOT_Bask.Shoot_Number;
        }

        AIM = (float) CONTAL->SHOOT.Shoot_Speed + TEMP;

    }
    return AIM;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-2-25 下午1:46
 *	@ReadMe:			//获取拨弹电机的目标值
 ************************************************************万能分隔符**************************************************************/
static int64_t RUI_F_GET_FIRE_AIM(DBUS_Typedef *DBUS,
                                  CONTAL_Typedef *CONTAL,
                                  User_Data_T *User_data)
{
    static uint8_t SINGLE_LOCK = 0;
    static int64_t AIM = 0;
    uint8_t MOD = DBUS->Remote.S1_u8-1 | DBUS->Mouse.L_State;

    //停止
    if (MOD == 0)
    {
        //单发解锁
        SINGLE_LOCK = 0;
        AIM = CONTAL->SHOOT_Bask.Angle;
    }
    else
    {
        if (MOD == 1 && SINGLE_LOCK == 0)
        {
            //单发上锁
            SINGLE_LOCK = 1;
            int64_t Temp = RUI_F_MATH_ABS_int64_t(CONTAL->SHOOT_Bask.Angle % CONTAL->SHOOT.Single_Angle);
            if (Temp > RUI_F_MATH_ABS_int64_t(CONTAL->SHOOT.Single_Angle) >> 1)
            {
                AIM = CONTAL->SHOOT_Bask.Angle - Temp;
            } else
            {
                AIM = CONTAL->SHOOT_Bask.Angle + CONTAL->SHOOT.Single_Angle + Temp;
            }
        }
        if (MOD == 2)//连发,简陋的火控
        {   //上限热量-当前热量<10,停火
            if(User_data->robot_status.shooter_barrel_heat_limit -
                    User_data->power_heat_data.shooter_17mm_1_barrel_heat < 10)
            {
                AIM = (int64_t)CONTAL->SHOOT_Bask.Angle;
            }
            else
            {
                AIM = (int64_t)CONTAL->SHOOT_Bask.Angle + CONTAL->SHOOT.Single_Angle;
            }
        }
    }
    return AIM;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-5-8 上午9:44
 *	@ReadMe:			//获取最大目标值
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_GET_MAX_TARGET(float MAX_POWER)
{
    // 200w 0.04f
    // 100w 0.065f
    //  90w 0.065f
    //  80w 0.07f
    //  75w 0.08f
    //  70w 0.09f
    //  65w 0.1f
    //  60w 0.08f
    //  55w 0.06f
    //  50w 0.04f
    //  45w 0.02f
    if (MAX_POWER == 45)
    {
        return 0.03f * MAX_POWER;
    } else if (MAX_POWER == 50 || MAX_POWER == 200)
    {
        return 0.04f * MAX_POWER;
    } else if (MAX_POWER == 55)
    {
        return 0.06f * MAX_POWER;
    } else if (MAX_POWER == 60 || MAX_POWER == 75)
    {
        return 0.08f * MAX_POWER;
    } else if (MAX_POWER == 65)
    {
        return 0.1f * MAX_POWER;
    } else if (MAX_POWER == 70)
    {
        return 0.09f * MAX_POWER;
    } else if (MAX_POWER == 80)
    {
        return 0.07f * MAX_POWER;
    } else if (MAX_POWER == 90 || MAX_POWER == 100)
    {
        return 0.065f * MAX_POWER;
    } else
    {
        return 0.1f * MAX_POWER;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //底盘走直线单环PID
 *	@parameter:		    //
 *	@time:				//23-12-17 18:08
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_PID(int16_t RELATIVE_ANGLE, float KP, float KI, float KD)
{
    static float INTEGRAL = 0.0;
    float ERROR[2] = { 0 }, DERIVATIVE;
    ERROR[ 1 ] = (float) RELATIVE_ANGLE;
    //积分
    INTEGRAL += ( ERROR[ 1 ] * KI );
    INTEGRAL = RUI_F_MATH_Limit_float(100, -100, INTEGRAL);

    //微分
    DERIVATIVE = ( ERROR[ 1 ] - ERROR[ 0 ] ) * KD;

    ERROR[ 0 ] = ERROR[ 1 ];
    float OUTPUT = RUI_F_MATH_Limit_float(3000, -3000, ( KP * ERROR[ 1 ] + INTEGRAL + DERIVATIVE ));

    return OUTPUT;
}
