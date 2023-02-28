/*
 * can_it.c
 *
 *  Created on: 2023. 2. 21.
 *      Author: KSB
 */

#include "can_it.h"
#include <stdio.h>

extern CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef CAN_TXH;
CAN_RxHeaderTypeDef CAN_RXH;
CAN_FilterTypeDef filtername;
uint32_t Mailbox;
uint8_t TXData[8] = {8, 7, 6, 5, 4, 3, 2, 1};
uint8_t RXData[8] = {0};
uint8_t C0Data[8] = {0};

void CAN_RX_Header_defunc()
{
	filtername.FilterActivation = CAN_FILTER_ENABLE; // filter on,off
	filtername.FilterBank = 1; // filterbank initialize single can = 0~13, dual can = 0~27
	filtername.FilterFIFOAssignment = CAN_FILTER_FIFO0; // fifo assgin 0 or 1
	filtername.FilterIdHigh = 0x0000; //
	filtername.FilterIdLow = 0x0000;
	filtername.FilterMaskIdHigh = 0x0000;
	filtername.FilterMaskIdLow = 0x0000;
	filtername.FilterMode = CAN_FILTERMODE_IDMASK; // filter mode -> mask or list
	filtername.FilterScale = CAN_FILTERSCALE_16BIT; // filter scale
	// filtername.SlaveStartFilterBank // only dual can

	HAL_CAN_ConfigFilter(&hcan1, &filtername);
}

void CAN_TX_Header_defunc()
{
	CAN_TXH.DLC = 8;
	CAN_TXH.IDE = CAN_ID_STD;
	CAN_TXH.RTR = CAN_RTR_DATA;
	CAN_TXH.StdId = 0xc0;
	CAN_TXH.TransmitGlobalTime = DISABLE;
}

void CAN_Error_Handler()
{
	if (HAL_CAN_ConfigFilter(&hcan1, &filtername) != HAL_OK)
	{
		// Filter configuration Error
		Error_Handler();
	}

	// Can Start
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		// Start Error
		Error_Handler();
	}

	// Activate CAN RX notification
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK)
	{
		// Notification Error
		Error_Handler();
	}
}

void callbackSystick() //1ms count
{
	static int count = 0;
	count++;
	if (count == 10)  //10ms send can tx message
	{
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		{
			HAL_CAN_AddTxMessage(&hcan1, &CAN_TXH, TXData, &Mailbox);
		}
		count = 0;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	if(HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &CAN_RXH, RXData) != HAL_OK)
    {
      Error_Handler();
    }
}
