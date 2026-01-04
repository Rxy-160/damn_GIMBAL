/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-07-18 16:44:18
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-18 16:48:46
 */
#include "can_bsp.h"

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //CAN ID过滤
 *	@parameter:		    //
 *	@time:				//22-11-23 20:39
 *	@ReadMe:			//放在主函数里初始化一次
 ************************************************************万能分隔符**************************************************************/
void CAN_Filter_Init(void)
{
	CAN_FilterTypeDef CAN_FilterInitStrt;
	
    CAN_FilterInitStrt.SlaveStartFilterBank = 14;
    CAN_FilterInitStrt.FilterBank           = 14;
    CAN_FilterInitStrt.FilterActivation     = ENABLE;
    CAN_FilterInitStrt.FilterMode           = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStrt.FilterScale          = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStrt.FilterIdHigh         = 0x0000;
    CAN_FilterInitStrt.FilterIdLow          = 0x0000;
    CAN_FilterInitStrt.FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStrt.FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStrt.FilterBank           = 0;
    CAN_FilterInitStrt.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1 , &CAN_FilterInitStrt);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1 , CAN_IT_RX_FIFO0_MSG_PENDING);

    CAN_FilterInitStrt.SlaveStartFilterBank = 14;
    CAN_FilterInitStrt.FilterBank           = 14;

    HAL_CAN_ConfigFilter(&hcan2 , &CAN_FilterInitStrt);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2 , CAN_IT_RX_FIFO0_MSG_PENDING);

    return;
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//小瑞
 *	@performance:	    //共用体can发送
 *	@parameter:		    //
 *	@time:				//23-05-06 19:07
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
void canx_send_data(CAN_HandleTypeDef* _hcan , uint16_t stdid , uint8_t* Data)
{
    CAN_TxHeaderTypeDef TXMessage;

    uint32_t send_mail_box;

    TXMessage.DLC   = 0x08;
    TXMessage.IDE   = CAN_ID_STD;
    TXMessage.RTR   = CAN_RTR_DATA;
    TXMessage.StdId = stdid;

    HAL_CAN_AddTxMessage(_hcan , &TXMessage , Data , &send_mail_box);
}
