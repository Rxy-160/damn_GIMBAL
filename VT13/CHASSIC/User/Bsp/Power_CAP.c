
#include "Power_CAP.h"
int opencap=0;
int powerlimit=0;
/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//2024/1/12 16:51
 *	@ReadMe:			//电容接收解算函数
 ************************************************************万能分隔符**************************************************************/
void Power_CAP_CAN_RX(CAPDATE_TYPDEF *CAP_DATA, uint8_t *DATA)
{
	  typedef union {
		float float_data;
		uint8_t data[4];
	} typedef_P_CHASS_DATA;
		typedef_P_CHASS_DATA  power;
    CAP_DATA->GET.cap_key   =  DATA[0] ;//电容是否开关
    CAP_DATA->GET.cap_state =  DATA[ 1] ;//错误码（过压过流）0,1,2,3,4,5；
    power.data[0]= DATA[ 2];
	  power.data[1]= DATA[ 3];
	  power.data[2]= DATA[ 4];
	  power.data[3]= DATA[ 5];
	  CAP_DATA->GET.nowPower= power.float_data;
    CAP_DATA->GET.CAP_cap=DATA[6] ;
	  CAP_DATA->GET.capVolt=DATA[7]; 
}

void Power_CAP_CAN_TX(hcan_t* hcan,uint16_t cap_id,CAPDATE_TYPDEF *CAP_DATA, User_Data_T *User_data)
{
    CAP_DATA->SET.CHANNAL.power_key = opencap;
    CAP_DATA->SET.CHANNAL.capPowerLimit = powerlimit;
    CAP_DATA->SET.CHANNAL.BUFFER_NOW =User_data->power_heat_data.buffer_energy  ;
    CAP_DATA->SET.CHANNAL.robot_state = 1;
	  CAP_DATA->SET.CHANNAL.check_code = 0xAA;
    canx_send_data(hcan, cap_id,  CAP_DATA->SET.SEND_DATA);
}
