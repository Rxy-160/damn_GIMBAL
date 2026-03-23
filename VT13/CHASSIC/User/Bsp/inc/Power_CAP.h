#ifndef __POWER_CAP_H
#define __POWER_CAP_H

#include "main.h"
#include "MY_Define.h"
#include "can_bsp.h"
#include "Referee.h"


typedef struct  // 接收的电容的数据
{
    uint8_t cap_key;  //FSBB开关
	  uint8_t cap_state;//电容状态
		float buffer; 		//电容缓冲
		float capVolt;		//电容电压
		float nowPower;		//当前总功率
		float bus_voltage; //总线电压
		uint8_t CAP_cap; //电容容量
	  float I;           //电容电流
	  float   cap_realy_out;    //电容的净输出
}CAP_RXDATA;

typedef union  // 发给电容的数据//使用共用体整合数据
{
    struct __packed
    { 
			 uint8_t power_key:8;        //是否开关电容
			 uint8_t capPowerLimit :8;		//功率限制
       uint8_t BUFFER_NOW; // 当前缓冲
			 uint8_t robot_state :8;			//机器人状态
			 uint8_t check_code:8;		//固定校验
    }CHANNAL;
    // CAN发送的数据
    uint8_t SEND_DATA[5];
}CAP_SETDATA;

typedef struct
{
    CAP_SETDATA SET;
    CAP_RXDATA GET;
} CAPDATE_TYPDEF;
void Power_CAP_CAN_RX(CAPDATE_TYPDEF *CAP_DATA, uint8_t *DATA);
void Power_CAP_CAN_TX(hcan_t* hcan,uint16_t cap_id,CAPDATE_TYPDEF *CAP_DATA, User_Data_T *User_data);

#endif
