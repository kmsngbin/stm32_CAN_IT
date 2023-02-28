/*
 * can_it.h
 *
 *  Created on: 2023. 2. 21.
 *      Author: KSB
 */

#ifndef INC_CAN_IT_H_
#define INC_CAN_IT_H_

#include "main.h"
#include "can.h"
#include "stm32l4xx_it.h"

void CAN_RX_Header_defunc();
void CAN_TX_Header_defunc();
void CAN_Error_Handler();
void callbackSystick();
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);

extern uint8_t RXData[8];

#endif /* INC_CAN_IT_H_ */
