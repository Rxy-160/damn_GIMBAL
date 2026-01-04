#ifndef _NIMING_H_
#define _NIMING_H_

#include "usart.h"
#include "usbd_cdc_if.h"

#define df_AnonymityLong 70
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

extern uint8_t anonymity_au8[df_AnonymityLong];												//ÄäÃûÓÃÊı×é

void Anonymity_TX(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);
	
#endif
