/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2023-09-13 09:29:47
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-18 16:49:07
 */
#ifndef __CAN_BSP_H
#define __CAN_BSP_H

#include "can.h"
#include "string.h"
#include "MY_define.h"

typedef CAN_HandleTypeDef hcan_t;

void CAN_Filter_Init(void);

//共用体
void canx_send_data(CAN_HandleTypeDef* _hcan , uint16_t stdid , uint8_t* Data);

#endif
