#ifndef __WHW_IRQN_H
#define __WHW_IRQN_H

#include "All_Init.h"
#include "Vision.h"
extern void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim);
extern void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);
void DWT_DelayUs(uint32_t us);

#endif
