/*
 * @Descripttion: 
 * @version: 
 * @Author: Eugene
 * @Date: 2024-06-29 13:17:29
 * @LastEditors: Andy
 * @LastEditTime: 2024-07-05 15:40:08
 */
#include "NIMING.h"

/************************************************************变量乱葬岗**************************************************************/
uint8_t anonymity_au8[df_AnonymityLong] = {0};

/************************************************************万能分隔符**************************************************************
*	@author:			//赵澍
*	@performance:	//匿名发送用于观看数据曲线
*	@parameter:		//太简单//NO
*	@time:				//21-1-6
*	@ReadMe:			//协议见具体内容
************************************************************************************************************************************/
void Anonymity_TX(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	//匿名V7发送协议
    uint8_t _cnt=0,sum=0,add=0,i=0;
    int16_t _temp;

    anonymity_au8[_cnt++]=0xAA;
    anonymity_au8[_cnt++]=0xFF;
    anonymity_au8[_cnt++]=0xF1;
    anonymity_au8[_cnt++]=0;//176

    _temp = a_x;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = a_y;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = a_z;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);

    _temp = g_x;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = g_y;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = g_z;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);

    _temp = m_x;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = m_y;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);
    _temp = m_z;
    anonymity_au8[_cnt++]=BYTE0(_temp);
    anonymity_au8[_cnt++]=BYTE1(_temp);

    anonymity_au8[3] = _cnt-4;

    for(i=0; (i<anonymity_au8[3]+4); i++)
    {
            sum += anonymity_au8[i];
            add += sum;
    }
    anonymity_au8[_cnt++]=sum;
    anonymity_au8[_cnt++]=add;

    CDC_Transmit_FS(anonymity_au8,70);

}
