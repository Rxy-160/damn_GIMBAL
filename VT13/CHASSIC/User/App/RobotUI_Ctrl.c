#include "RobotUI_Ctrl.h"

uint8_t RobotUI_Static_Init()
{	
		ui_init_g_1();
	osDelay(40);
	return RUI_DF_READY;
}

void RobotUI_Dynamic()
{
    static uint8_t UI_Init = RUI_DF_READY;

//    //遥控离线监测
    if(CanCommunit_t.gmTOch .dataNeaten .vx )
    {
        UI_Init = RUI_DF_ERROR;
    }
		
    //静态UI刷新
    if (UI_Init != RUI_DF_READY)
    { 
        UI_Init = RobotUI_Static_Init();
    }
		ui_update_g_1();

		
}
