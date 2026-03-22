/*
 * ......................................&&.........................
 * ....................................&&&..........................
 * .................................&&&&............................
 * ...............................&&&&..............................
 * .............................&&&&&&..............................
 * ...........................&&&&&&....&&&..&&&&&&&&&&&&&&&........
 * ..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............
 * ................&...&&&&&&&&&&&&&&&&&&&&&&&&&&&&.................
 * .......................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........
 * ...................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...............
 * ..................&&&   &&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * ...............&&&&&@  &&&&&&&&&&..&&&&&&&&&&&&&&&&&&&...........
 * ..............&&&&&&&&&&&&&&&.&&....&&&&&&&&&&&&&..&&&&&.........
 * ..........&&&&&&&&&&&&&&&&&&...&.....&&&&&&&&&&&&&...&&&&........
 * ........&&&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&....&&&.......
 * .......&&&&&&&&.....................&&&&&&&&&&&&&&&&.....&&......
 * ........&&&&&.....................&&&&&&&&&&&&&&&&&&.............
 * ..........&...................&&&&&&&&&&&&&&&&&&&&&&&............
 * ................&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&............
 * ..................&&&&&&&&&&&&&&&&&&&&&&&&&&&&..&&&&&............
 * ..............&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&....&&&&&............
 * ...........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&......&&&&............
 * .........&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&.........&&&&............
 * .......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&............
 * ......&&&&&&&&&&&&&&&&&&&...&&&&&&...............&&&.............
 * .....&&&&&&&&&&&&&&&&............................&&..............
 * ....&&&&&&&&&&&&&&&.................&&...........................
 * ...&&&&&&&&&&&&&&&.....................&&&&......................
 * ...&&&&&&&&&&.&&&........................&&&&&...................
 * ..&&&&&&&&&&&..&&..........................&&&&&&&...............
 * ..&&&&&&&&&&&&...&............&&&.....&&&&...&&&&&&&.............
 * ..&&&&&&&&&&&&&.................&&&.....&&&&&&&&&&&&&&...........
 * ..&&&&&&&&&&&&&&&&..............&&&&&&&&&&&&&&&&&&&&&&&&.........
 * ..&&.&&&&&&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&&&&&&&&&&&&.......
 * ...&&..&&&&&&&&&&&&.........&&&&&&&&&&&&&&&&...&&&&&&&&&&&&......
 * ....&..&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&...........&&&&&&&&.....
 * .......&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&..............&&&&&&&....
 * .......&&&&&.&&&&&&&&&&&&&&&&&&..&&&&&&&&...&..........&&&&&&....
 * ........&&&.....&&&&&&&&&&&&&.....&&&&&&&&&&...........&..&&&&...
 * .......&&&........&&&.&&&&&&&&&.....&&&&&.................&&&&...
 * .......&&&...............&&&&&&&.......&&&&&&&&............&&&...
 * ........&&...................&&&&&&.........................&&&..
 * .........&.....................&&&&........................&&....
 * ...............................&&&.......................&&......
 * ................................&&......................&&.......
 * .................................&&..............................
 * ..................................&..............................
 */


#include "RUI_DBUS.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//  小瑞 COPY FORM 赵澍
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//  22-11-21 18:47
 *	@ReadMe:	        //  这个版本有遥控滚轮的控制，但是某些遥控没有办法正确传回滚轮数据
                        //	遥控通道说明图
                        //	^						^
                        //	|						|
                        //	|ch2---->		        |ch0---->
                        //	|						|
                        //	ch3					    ch1
                        //  共用体接收
 ************************************************************万能分隔符**************************************************************/
void RUI_F_DUBS_Resovled(uint8_t* Data, DBUS_Typedef *RUI_V_DBUS)
{
    RUI_V_DBUS->RUI_V_DBUS_ONLINE_JUDGE_TIME = RUI_DF_DBUS_OFFLINE_TIME;
    DBUS_UNION_Typdef RUI_V_DBUS_UNION = { 0 };

    static uint8_t Key_Q_Lock = 0; // 0是开锁，1是上锁
    static uint8_t Key_E_Lock = 0;
    static uint8_t Key_R_Lock = 0;
    static uint8_t Key_F_Lock = 0;
    static uint8_t Key_G_Lock = 0;
    static uint8_t Key_Z_Lock = 0;
    static uint8_t Key_X_Lock = 0;
    static uint8_t Key_C_Lock = 0;
    static uint8_t Key_V_Lock = 0;
    static uint8_t Key_B_Lock = 0;
    static uint8_t Key_Shift_Lock = 0;
    static uint8_t Key_Ctrl_Lock = 0;
    static uint8_t Mouse_R_Lock = 0;
    static uint8_t Mouse_L_Lock = 0;

    memcpy(RUI_V_DBUS_UNION.GetData , Data , 19);

    RUI_V_DBUS->Remote.S1_u8 = RUI_V_DBUS_UNION.DataNeaten.S1;
    RUI_V_DBUS->Remote.S2_u8 = RUI_V_DBUS_UNION.DataNeaten.S2;

    RUI_V_DBUS->Remote.CH0_int16  = RUI_V_DBUS_UNION.DataNeaten.CH0 -1024;
    RUI_V_DBUS->Remote.CH1_int16  = RUI_V_DBUS_UNION.DataNeaten.CH1 -1024;
    RUI_V_DBUS->Remote.CH2_int16  = RUI_V_DBUS_UNION.DataNeaten.CH2 -1024;
    RUI_V_DBUS->Remote.CH3_int16  = RUI_V_DBUS_UNION.DataNeaten.CH3 -1024;
    RUI_V_DBUS->Remote.Dial_int16 = RUI_V_DBUS_UNION.DataNeaten.Direction -1024;

    if (RUI_V_DBUS_UNION.DataNeaten.CH0 == 0)
    {
        RUI_V_DBUS->Remote.CH0_int16  = 0;
        RUI_V_DBUS->Remote.CH1_int16  = 0;
        RUI_V_DBUS->Remote.CH2_int16  = 0;
        RUI_V_DBUS->Remote.CH3_int16  = 0;
        RUI_V_DBUS->Remote.Dial_int16 = 0;
    }

    //*对点按和长按的区分*//
    RUI_V_DBUS->Mouse.R_State = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.Mouse_R , RUI_V_DBUS->Mouse.R_PressTime);
    RUI_V_DBUS->Mouse.L_State = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.Mouse_L , RUI_V_DBUS->Mouse.L_PressTime);

    //*键盘的按键*//
    RUI_V_DBUS->KeyBoard.W = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.KeyBoard_W , RUI_V_DBUS->KeyBoard.W_PressTime);
    RUI_V_DBUS->KeyBoard.A = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.KeyBoard_A , RUI_V_DBUS->KeyBoard.A_PressTime);
    RUI_V_DBUS->KeyBoard.S = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.KeyBoard_S , RUI_V_DBUS->KeyBoard.S_PressTime);
    RUI_V_DBUS->KeyBoard.D = RUI_F_KEY_STATUS(RUI_V_DBUS_UNION.DataNeaten.KeyBoard_D , RUI_V_DBUS->KeyBoard.D_PressTime);

    //*鼠标滤波*//
    RUI_V_DBUS->Mouse.X_Flt = OneFilter(RUI_V_DBUS->Mouse.X_Flt , (float) RUI_V_DBUS_UNION.DataNeaten.Mouse_X , 500);
    RUI_V_DBUS->Mouse.Y_Flt = OneFilter(RUI_V_DBUS->Mouse.Y_Flt , (float) RUI_V_DBUS_UNION.DataNeaten.Mouse_Y , 500);

    // Shift
    RUI_V_DBUS->KeyBoard.Shift = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_Shift;
    if (RUI_V_DBUS->KeyBoard.Shift == 1 && Key_Shift_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.Shift_PreeNumber = !RUI_V_DBUS->KeyBoard.Shift_PreeNumber;
        Key_Shift_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.Shift == 0 && Key_Shift_Lock == 1)
    {
        Key_Shift_Lock = 0; // 开锁
    }
    // Ctrl
    RUI_V_DBUS->KeyBoard.Ctrl = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_Ctrl;
    if (RUI_V_DBUS->KeyBoard.Ctrl == 1 && Key_Ctrl_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.Ctrl_PreeNumber = !RUI_V_DBUS->KeyBoard.Ctrl_PreeNumber;
        Key_Ctrl_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.Ctrl == 0 && Key_Ctrl_Lock == 1)
    {
        Key_Ctrl_Lock = 0; // 开锁
    }
    // Q
    RUI_V_DBUS->KeyBoard.Q = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_Q;
    if (RUI_V_DBUS->KeyBoard.Q == 1 && Key_Q_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.Q_PreeNumber = !RUI_V_DBUS->KeyBoard.Q_PreeNumber;
        Key_Q_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.Q == 0 && Key_Q_Lock == 1)
    {
        Key_Q_Lock = 0; // 开锁
    }
    // E
    RUI_V_DBUS->KeyBoard.E = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_E;
    if (RUI_V_DBUS->KeyBoard.E == 1 && Key_E_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.E_PreeNumber = !RUI_V_DBUS->KeyBoard.E_PreeNumber;
        Key_E_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.E == 0 && Key_E_Lock == 1)
    {
        Key_E_Lock = 0; // 开锁
    }
    // R
    RUI_V_DBUS->KeyBoard.R = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_R;
    if (RUI_V_DBUS->KeyBoard.R == 1 && Key_R_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.R_PreeNumber = !RUI_V_DBUS->KeyBoard.R_PreeNumber;
        Key_R_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.R == 0 && Key_R_Lock == 1)
    {
        Key_R_Lock = 0; // 开锁
    }
    // F
    RUI_V_DBUS->KeyBoard.F = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_F;
    if (RUI_V_DBUS->KeyBoard.F == 1 && Key_F_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.F_PreeNumber = !RUI_V_DBUS->KeyBoard.F_PreeNumber;
        Key_F_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.F == 0 && Key_F_Lock == 1)
    {
        Key_F_Lock = 0; // 开锁
    }
    // Z
    RUI_V_DBUS->KeyBoard.Z = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_Z;
    if (RUI_V_DBUS->KeyBoard.Z == 1 && Key_Z_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.Z_PreeNumber = !RUI_V_DBUS->KeyBoard.Z_PreeNumber;
        Key_Z_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.Z == 0 && Key_Z_Lock == 1)
    {
        Key_Z_Lock = 0; // 开锁
    }
    // G
    RUI_V_DBUS->KeyBoard.G = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_G;
    if (RUI_V_DBUS->KeyBoard.G == 1 && Key_G_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.G_PreeNumber = !RUI_V_DBUS->KeyBoard.G_PreeNumber;
        Key_G_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.G == 0 && Key_G_Lock == 1)
    {
        Key_G_Lock = 0; // 开锁
    }
    // X
    RUI_V_DBUS->KeyBoard.X = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_X;
    if (RUI_V_DBUS->KeyBoard.X == 1 && Key_X_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.X_PreeNumber = !RUI_V_DBUS->KeyBoard.X_PreeNumber;
        Key_X_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.X == 0 && Key_X_Lock == 1)
    {
        Key_X_Lock = 0; // 开锁
    }
    // C
    RUI_V_DBUS->KeyBoard.C = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_C;
    if (RUI_V_DBUS->KeyBoard.C == 1 && Key_C_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.C_PreeNumber = !RUI_V_DBUS->KeyBoard.C_PreeNumber;
        Key_C_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.C == 0 && Key_C_Lock == 1)
    {
        Key_C_Lock = 0; // 开锁
    }

    RUI_V_DBUS->KeyBoard.V = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_V;
	if (RUI_V_DBUS->KeyBoard.V == 1 && Key_V_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.V_PreeNumber = !RUI_V_DBUS->KeyBoard.V_PreeNumber;
        Key_V_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.V == 0 && Key_V_Lock == 1)
    {
        Key_V_Lock = 0; // 开锁
    }
	
    RUI_V_DBUS->KeyBoard.B = RUI_V_DBUS_UNION.DataNeaten.KeyBoard_B;
	if (RUI_V_DBUS->KeyBoard.B == 1 && Key_B_Lock == 0)
    {
        RUI_V_DBUS->KeyBoard.B_PreeNumber = !RUI_V_DBUS->KeyBoard.B_PreeNumber;
        Key_B_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->KeyBoard.B == 0 && Key_B_Lock == 1)
    {
        Key_B_Lock = 0; // 开锁
    }
	
	RUI_V_DBUS->Mouse.L_State = RUI_V_DBUS_UNION.DataNeaten.Mouse_L;
	if (RUI_V_DBUS->Mouse.L_State == 1 && Mouse_L_Lock == 0)
    {
        RUI_V_DBUS->Mouse.L_PressTime = !RUI_V_DBUS->Mouse.L_PressTime;
        Mouse_L_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->Mouse.L_State == 0 && Mouse_L_Lock == 1)
    {
        Mouse_L_Lock = 0; // 开锁
    }
	
	RUI_V_DBUS->Mouse.R_State = RUI_V_DBUS_UNION.DataNeaten.Mouse_R;
	if (RUI_V_DBUS->Mouse.R_State == 1 && Mouse_R_Lock == 0)
    {
        RUI_V_DBUS->Mouse.R_PressTime = !RUI_V_DBUS->Mouse.R_PressTime;
        Mouse_R_Lock = 1; // 上锁
    }
    else if (RUI_V_DBUS->Mouse.R_State == 0 && Mouse_R_Lock == 1)
    {
        Mouse_R_Lock = 0; // 开锁
    }
}


/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞 COPY 赵澍
 *	@performance:	    //鼠标滤波
 *	@parameter:		    //上一次的值//当前值//尖峰敏感度
 *	@time:				//22-11-23 16:39
 *	@ReadMe:			//使用一阶低通滤波(改进版)
                        //尖峰敏感度：越小对尖峰越敏感	(一般取值为最大值的20%)
 ************************************************************万能分隔符**************************************************************/
float OneFilter(float last , float now , float thresholdValue)
{
    // 减小平滑滤波值会增大对于细小毛刺的过滤程度
    // 增加尖峰滤波值会增大对于尖峰数值的响应程度
    const float sensitivlFilter = 0.8f; // 尖峰滤波值//小于1
    const float numbFilter = 0.2f; // 平滑滤波值//小于1

    if (RUI_F_MATH_ABS_float(RUI_F_MATH_ABS_float(last) - RUI_F_MATH_ABS_float(now)) >= thresholdValue)
    {
        return (float) (now * sensitivlFilter + last * (1 - sensitivlFilter));
    }
    else
    {
        return (float) (now * numbFilter + last * (1 - numbFilter));
    }
}
/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //按键长按短按
 *	@parameter:		    //
 *	@time:				//23-04-26 21:40
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
uint8_t RUI_F_KEY_STATUS(uint64_t  KEY , uint8_t PRESS_TIME)
{
    if (KEY == 1)
    {
        if (PRESS_TIME <= 20)
        {
            PRESS_TIME++;
            return RUI_DF_KEY_CLICK;
        }
        else
        {
            return  RUI_DF_KEY_PRESS; // 长按
        }
    }
    else
    {
        PRESS_TIME = 0;
        return  RUI_DF_KEY_UP;
    }

}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//提灿
 *	@performance:	    //遥控键鼠转化函数
 *	@parameter:		    //
 *	@time:				//24-12-10 16:37
 *	@ReadMe:			//这个可以说是一个十分核心的函数
 ************************************************************万能分隔符**************************************************************/
void F_DUBS_CONTROL(DBUS_Typedef *vDbus , CONTAL_Typedef *V_CONTAL)
{
	
	switch (vDbus->Remote.S1_u8 + vDbus->Remote.S2_u8 * 3)
	{	
/***************************************头部begin**************************************************/				
						//		S1	 + 	S2
		case 0x04:	    //		1		1
		{ 
			
		}
        break;
		case 0x0a:	    //		3		1
		{
            
		}
        break;
		case 0x07:		//		2		1
		{
			
		}
        break;
/***************************************头部end**************************************************/

		
/***************************************中部begin**************************************************/		
		
		
		case 0x06:		//		1		3
		{
			
		}	
        break;
		
		case 0x0c:		//		3		3
		{

		}	
        break;
		
		case 0x09:		//		2		3
		{

		}	
        break;
			
/***************************************中部end**************************************************/
		

/***************************************底部begin**************************************************/
		case 0x05:		//		1		2
		{

		}	
        break;
		
		case 0x0b:		//		3		2
		{
			
		}	
        break;
		
		case 0x08:		//		2		2		键鼠模式
		{
			
		}	
        break;
		
/***************************************底部end**************************************************/
	}
/***************************************键鼠***************************************************/

}
