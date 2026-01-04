#include "RGBctrl.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //LED
 *	@parameter:		    //
 *	@time:				//23-04-13 16:05
 *	@ReadMe:			//颜色参考RUI_DEFINE.h文件中的颜色 TIME小于10时视觉效果为长亮
 ************************************************************万能分隔符**************************************************************/
void RUI_F_LED(uint8_t COLOR , int TIME)
{
	static uint32_t TIME_LAST = 0;
	
    uint32_t TIME_NOW = (uint32_t)DWT_GetTimeline_ms();

    static int8_t BIT = 0 , status_LAST = 9;

    static GPIO_TypeDef* LED_PORT[3] ={ LED_R_GPIO_Port, LED_G_GPIO_Port, LED_B_GPIO_Port };

    static uint16_t LED_PIN[3] ={ LED_R_Pin, LED_G_Pin, LED_B_Pin };

    static uint8_t LED_CONFIG[6][3] ={
        {1, 0, 0},    // RED
        {0, 1, 0},    // GREEN
        {0, 0, 1},    // BLUE
        {1, 1, 0},    // YELLOW
        {1, 0, 1},    // PURPLE
        {0, 1, 1},    // CYAN
    };

    if (COLOR != status_LAST)
    {
        for (int i = 0; i < 3; i++)
        {
            HAL_GPIO_WritePin(LED_PORT[i] , LED_PIN[i] , GPIO_PIN_SET);
        }
        status_LAST = (int8_t)COLOR;
    }

    if ((TIME_NOW - TIME_LAST) > TIME)
    {
        BIT = !BIT;
        TIME_LAST = TIME_NOW;
    }

    for (int i = 0; i < 3; i++)
    {
        if (LED_CONFIG[COLOR][i])
        {
			if(BIT == 0)
				HAL_GPIO_WritePin(LED_PORT[i] , LED_PIN[i] , GPIO_PIN_RESET);
			if(BIT > 0)
				HAL_GPIO_WritePin(LED_PORT[i] , LED_PIN[i] , GPIO_PIN_SET);
        }
    }
}

