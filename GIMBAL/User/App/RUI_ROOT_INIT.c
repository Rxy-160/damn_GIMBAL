/*
 *                                                     __----~~~~~~~~~~~------___
 *                                    .  .   ~~//====......          __--~ ~~
 *                    -.            \_|//     |||\\  ~~~~~~::::... /~
 *                 ___-==_       _-~o~  \/    |||  \\            _/~~-
 *         __---~~~.==~||\=_    -_--~/_-~|-   |\\   \\        _/~
 *     _-~~     .=~    |  \\-_    '-~7  /-   /  ||    \      /
 *   .~       .~       |   \\ -_    /  /-   /   ||      \   /
 *  /  ____  /         |     \\ ~-_/  /|- _/   .||       \ /
 *  |~~    ~~|--~~~~--_ \     ~==-/   | \~--===~~        .\
 *           '         ~-|      /|    |-~\~~       __--~~
 *                       |-~~-_/ |    |   ~\_   _-~            /\
 *                            /  \     \__   \/~                \__
 *                        _--~ _/ | .-~~____--~-/                  ~~==.
 *                       ((->/~   '.|||' -_|    ~~-/ ,              . _||
 *                                  -_     ~\      ~~---l__i__i__i--~~_/
 *                                  _-~-__   ~)  \--______________--~~
 *                                //.-~~~-~_--~- |-------~~~~~~~~
 *                                       //.-~~~--\
 *                       ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *                               神兽保佑            永无BUG
 */
#include "RUI_ROOT_INIT.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //ROOT函数
 *	@parameter:		    //
 *	@time:				//23-04-13 17:38
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void RUI_F_ROOT(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET)
{
    RUI_F_ROOT_ALL_STATUS(Root, DBUS, MOTOR, CAP_GET);
    RUI_F_MASTER_LOCATION_JUDGE(Root);
    RUI_F_LED_SHOW_STATUS(Root);
}

/************************************************************万能分隔符**************************************************************
// * 	@author:			//WHW
// *	@performance:	    //大疆电机离线检测
// *	@parameter:		    //
// *	@time:				//23-04-13 15:01
// *	@ReadMe:			//
// ************************************************************万能分隔符**************************************************************/
uint8_t DJI_F_MOTOR_STATUS(DJI_MOTOR_DATA_Typedef* DATA)
{
    DATA->ONLINE_JUDGE_TIME--;

    if (DATA->ONLINE_JUDGE_TIME < 5)
    {
        DATA->ONLINE_JUDGE_TIME = 0;
        return RUI_DF_OFFLINE;
    }
    else
        return RUI_DF_ONLINE;
}

/************************************************************万能分隔符**************************************************************
// * 	@author:			//WHW
// *	@performance:	    //达妙电机离线检测
// *	@parameter:		    //
// *	@time:				//23-04-13 15:01
// *	@ReadMe:			//
************************************************************万能分隔符**************************************************************/
uint8_t DM_F_MOTOR_STATUS(DM_MOTOR_DATA_Typdef* DATA)
{
  DATA->ONLINE_JUDGE_TIME--;

  if (DATA->ONLINE_JUDGE_TIME < 5)
  {
    DATA->ONLINE_JUDGE_TIME = 0;
    return RUI_DF_OFFLINE;
  }
  else
    return RUI_DF_ONLINE;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //总的离线检测
 *	@parameter:		    //
 *	@time:				//23-03-29 19:34
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-04-13 15:22    bestrui
 *  @UpData：           //电机检测封装成函数
 ************************************************************万能分隔符**************************************************************/
void RUI_F_ROOT_ALL_STATUS(RUI_ROOT_STATUS_Typedef *Root, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR, CAP_RXDATA *CAP_GET)
{
    //遥控在线监测
    if (DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME < 5)
    {
        DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME = 3;
        Root->RM_DBUS = RUI_DF_OFFLINE;
    }
    else
    {
        Root->RM_DBUS = RUI_DF_ONLINE;
    }
    DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME--;

    //电容在线监测
    if(CAP_GET->ONLINE_JUDGE_TIME < 5)
    {
        CAP_GET->ONLINE_JUDGE_TIME = 3;
        Root->Power = RUI_DF_OFFLINE;
    }
    else
    {
        Root->Power = RUI_DF_ONLINE;
    }

    //电机在线检测
//    Root->MOTOR_HEAD_Pitch = DJI_F_MOTOR_STATUS(&MOTOR->DJI_6020_Pitch.DATA);
//    Root->MOTOR_HEAD_Yaw = DJI_F_MOTOR_STATUS(&MOTOR->DJI_6020_Yaw.DATA);
		Root->MOTOR_HEAD_Yaw =DM_F_MOTOR_STATUS (&MOTOR->m_dm4310_y_t   .DATA);
		Root->MOTOR_HEAD_Pitch =DM_F_MOTOR_STATUS (&MOTOR->m_dm4310_p_t .DATA );
    Root->MOTOR_Shoot_L = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Shoot_L.DATA);
    Root->MOTOR_Shoot_R = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Shoot_R.DATA);
    Root->MOTOR_Shoot_M = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Shoot_M.DATA);

    Root->MOTOR_Chassis_1 = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Chassis_1.DATA);
    Root->MOTOR_Chassis_2 = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Chassis_2.DATA);
    Root->MOTOR_Chassis_3 = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Chassis_3.DATA);
    Root->MOTOR_Chassis_4 = DJI_F_MOTOR_STATUS(&MOTOR->DJI_3508_Chassis_4.DATA);
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //主控位置判断
 *	@parameter:		    //
 *	@time:				//23-03-29 19:34
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t RUI_F_MASTER_LOCATION_JUDGE(RUI_ROOT_STATUS_Typedef *Root)
{
    Root->MASTER_LOCATION = RUI_DF_MASTER_LOCATION_HEAD;
    return RUI_DF_READY;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //用LED灯显示状态
 *	@parameter:		    //
 *	@time:				//23-04-13 17:36
 *	@ReadMe:			//
 *  @LastUpDataTime:    //2023-05-07 17:03    bestrui
 *  @UpData：           //底盘正常改为绿灯闪烁
 ************************************************************万能分隔符**************************************************************/
void RUI_F_LED_SHOW_STATUS(RUI_ROOT_STATUS_Typedef *Root)
{
    switch (Root->MASTER_LOCATION)
    {
        case RUI_DF_MASTER_LOCATION_JUDGEING:
            RUI_F_LED(RUI_DF_LED_YELLOW, 100);
        break;

        case RUI_DF_MASTER_LOCATION_HEAD:
        {
            if (Root->RM_DBUS           == RUI_DF_ONLINE &&
            Root->MOTOR_HEAD_Pitch      == RUI_DF_ONLINE &&
            Root->MOTOR_HEAD_Yaw        == RUI_DF_ONLINE &&
            Root->MOTOR_Chassis_1       == RUI_DF_ONLINE &&
            Root->MOTOR_Chassis_2       == RUI_DF_ONLINE &&
            Root->MOTOR_Chassis_3       == RUI_DF_ONLINE &&
            Root->MOTOR_Chassis_4       == RUI_DF_ONLINE &&
            Root->MOTOR_Shoot_L         == RUI_DF_ONLINE &&
            Root->MOTOR_Shoot_R         == RUI_DF_ONLINE &&
            Root->MOTOR_Shoot_M         == RUI_DF_ONLINE)
            {
                RUI_F_LED(RUI_DF_LED_CYAN, 200);
            }
            if (Root->RM_DBUS           == RUI_DF_OFFLINE &&
            (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
            Root->MOTOR_HEAD_Yaw        == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_1       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_2       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_3       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_4       == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_L         == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_R         == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_M	        == RUI_DF_OFFLINE))
            {
                RUI_F_LED(RUI_DF_LED_PURPLE, 50);
            }
            if (Root->RM_DBUS           == RUI_DF_ONLINE &&
            (Root->MOTOR_HEAD_Pitch     == RUI_DF_OFFLINE ||
            Root->MOTOR_HEAD_Yaw        == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_1       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_2       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_3       == RUI_DF_OFFLINE ||
            Root->MOTOR_Chassis_4       == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_L         == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_R         == RUI_DF_OFFLINE ||
            Root->MOTOR_Shoot_M	        == RUI_DF_OFFLINE))
            {
                RUI_F_LED(RUI_DF_LED_CYAN, 50);
            }
        }
        break;
    }
}
