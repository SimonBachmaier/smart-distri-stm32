/*
 * smart_distri.c
 *
 *  Created on: Nov 13, 2020
 *      Author: Simon
 */


#include "smart_distri.h"

/**
 * Setup all components
 */
void Setup()
{
	  HAL_ADC_Start_DMA(&hadc1, AdcValuesDmaBuffer, 8);

	  HAL_StatusTypeDef status = 0;
	  HAL_StatusTypeDef status2 = 0;
	  uint16_t callibration = 0xA00; //2560
	  uint8_t sendBuffer[2] = {0, 0};
	  sendBuffer[0] = callibration >> 8;
	  sendBuffer[1] = callibration;
	  uint8_t address = 0x40;
	  status =  HAL_I2C_Mem_Write(&hi2c1, address << 1, 0x05, I2C_MEMADD_SIZE_8BIT, sendBuffer, 2, 500);
	  uint8_t readBuffer[2] = {0, 0};
	  status2 =  HAL_I2C_Mem_Read(&hi2c1, address << 1, 0x05, I2C_MEMADD_SIZE_8BIT, readBuffer, 2, 500);

	  CAN1FilterConfig();
	  CAN1Start();
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	  HAL_TIM_Base_Start_IT(&htim1);
}

/**
 * Timer callback function
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM1) {
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);

	    GetFuseStatus();
	    SendAdcValues();
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if (hadc->Instance == ADC1)
//	{
//		for (uint8_t i = 0; i < 8; i++)
//		{
//			AdcValuesDma[i] = AdcValuesDmaBuffer[i];
//		}
//	}
//}

/**
 * CAN message receival interrupt callback
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader;
	/* Get RX message */
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}

	// Test id for restart test
	if (RxHeader.StdId == (CAN_MESSAGE_ID_START + CAN_MESSAGE_ID_RESTART_FUSES))
	{
		TryToRestartFuses();
	}
}
/**
 * Sends string up to 100 characters long to huart4
 */
void SendMsgToUart(char* msgText)
{
	  char msgOutUART[100];	//message for UART
	  //create UART message
	  memset(msgOutUART,'\0', 10);
	  strcat(msgOutUART, msgText);
	  strcat(msgOutUART, "\r\n");
	  //send UART message
	  if (HAL_UART_Transmit(&huart4, (uint8_t*)&msgOutUART, (uint16_t)sizeof(msgOutUART), 1000) != HAL_OK)
	  {
		  Error_Handler();
	  }
}

/**
 * Sends restart fuses test message
 */
void SendCANTestMessage()
{
	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t message = 'H';

	TxHeader.DLC = 1;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = CAN_MESSAGE_ID_START + CAN_MESSAGE_ID_RESTART_FUSES;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	// while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));
}

/**
 * Sends can message. Message id is added onto the base message id
 */
void SendCanMessage(uint8_t* data, uint32_t dataLengthInBits, uint8_t messageId)
{
	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	TxHeader.DLC = dataLengthInBits;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = CAN_MESSAGE_ID_START + messageId;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	// while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));
}

/**
 * Configures the can1 filter
 */
void CAN1FilterConfig()
{
	  CAN_FilterTypeDef  filterConfig;

	  filterConfig.FilterBank = 0;
	  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  filterConfig.FilterIdHigh = 0x0000;
	  filterConfig.FilterIdLow = 0x0000;
	  filterConfig.FilterMaskIdHigh = 0x0000;
	  filterConfig.FilterMaskIdLow = 0x0000;
	  filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  filterConfig.FilterActivation = ENABLE;
	  filterConfig.SlaveStartFilterBank = 14;

	  //activate CAN filter
	  if (HAL_CAN_ConfigFilter(&hcan1, &filterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
 * Starts can1
 */
void CAN1Start()
{
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	/* Start Error */
		Error_Handler();
	}
}

/**
 * Method tries to restart fuses by cycling SHDN pins
 */
void TryToRestartFuses()
{
	SendMsgToUart("TryToRestartFuses");

	HAL_GPIO_WritePin(FUSE1_SHDN_GPIO_Port, FUSE1_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE2_SHDN_GPIO_Port, FUSE2_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE3_SHDN_GPIO_Port, FUSE3_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE4_SHDN_GPIO_Port, FUSE4_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE5_SHDN_GPIO_Port, FUSE5_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE6_SHDN_GPIO_Port, FUSE6_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE7_SHDN_GPIO_Port, FUSE7_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE8_SHDN_GPIO_Port, FUSE8_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE9_SHDN_GPIO_Port, FUSE9_SHDN_Pin, GPIO_PIN_SET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(FUSE1_SHDN_GPIO_Port, FUSE1_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE2_SHDN_GPIO_Port, FUSE2_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE3_SHDN_GPIO_Port, FUSE3_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE4_SHDN_GPIO_Port, FUSE4_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE5_SHDN_GPIO_Port, FUSE5_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE6_SHDN_GPIO_Port, FUSE6_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE7_SHDN_GPIO_Port, FUSE7_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE8_SHDN_GPIO_Port, FUSE8_SHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FUSE9_SHDN_GPIO_Port, FUSE9_SHDN_Pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	HAL_GPIO_WritePin(FUSE1_SHDN_GPIO_Port, FUSE1_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE2_SHDN_GPIO_Port, FUSE2_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE3_SHDN_GPIO_Port, FUSE3_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE4_SHDN_GPIO_Port, FUSE4_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE5_SHDN_GPIO_Port, FUSE5_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE6_SHDN_GPIO_Port, FUSE6_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE7_SHDN_GPIO_Port, FUSE7_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE8_SHDN_GPIO_Port, FUSE8_SHDN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FUSE9_SHDN_GPIO_Port, FUSE9_SHDN_Pin, GPIO_PIN_SET);
}

/**
 * Checks fuse status and sends status per can message
 */
void GetFuseStatus()
{
	// FLT should be set if fuse is running and not set if fuse shut down
	GPIO_PinState fuseIsRunning[9] = { 0 };
	fuseIsRunning[0] = HAL_GPIO_ReadPin(FUSE1_FLT_GPIO_Port, FUSE1_FLT_Pin);
	fuseIsRunning[1] = HAL_GPIO_ReadPin(FUSE2_FLT_GPIO_Port, FUSE2_FLT_Pin);
	fuseIsRunning[2] = HAL_GPIO_ReadPin(FUSE3_FLT_GPIO_Port, FUSE3_FLT_Pin);
	fuseIsRunning[3] = HAL_GPIO_ReadPin(FUSE4_FLT_GPIO_Port, FUSE4_FLT_Pin);
	fuseIsRunning[4] = HAL_GPIO_ReadPin(FUSE5_FLT_GPIO_Port, FUSE5_FLT_Pin);
	fuseIsRunning[5] = HAL_GPIO_ReadPin(FUSE6_FLT_GPIO_Port, FUSE6_FLT_Pin);
	fuseIsRunning[6] = HAL_GPIO_ReadPin(FUSE7_FLT_GPIO_Port, FUSE7_FLT_Pin);
	fuseIsRunning[7] = HAL_GPIO_ReadPin(FUSE8_FLT_GPIO_Port, FUSE8_FLT_Pin);
	fuseIsRunning[8] = HAL_GPIO_ReadPin(FUSE9_FLT_GPIO_Port, FUSE9_FLT_Pin);

	uint8_t fuseIsRunningByteBuffer[2] = { 0, 0 };
	if (fuseIsRunning[0] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x80; // set 8th pin from LSB
	if (fuseIsRunning[1] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x40; // set 7th pin from LSB
	if (fuseIsRunning[2] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x20; // set 6th pin from LSB
	if (fuseIsRunning[3] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x10; // set 5th pin from LSB
	if (fuseIsRunning[4] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x8; // set 4th pin from LSB
	if (fuseIsRunning[5] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x4; // set third pin from LSB
	if (fuseIsRunning[6] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x2; // set second pin from LSB
	if (fuseIsRunning[7] == GPIO_PIN_SET) fuseIsRunningByteBuffer[0] &= 0x1; // set first pin from LSB
	if (fuseIsRunning[8] == GPIO_PIN_SET) fuseIsRunningByteBuffer[1] &= 0x80; // set 8th pin from LSB

	SendMsgToUart("About to send fuseIsRunningByteBuffer");
	SendCanMessage(fuseIsRunningByteBuffer, 2, CAN_MESSAGE_ID_FUSE_STATUS);
}

/**
 * Sends adc values from adc buffer as can messages
 */
void SendAdcValues() {
	SendMsgToUart("About to send all read adc values");
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[0]), 4, 1);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[1]), 4, 2);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[2]), 4, 3);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[3]), 4, 4);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[4]), 4, 5);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[5]), 4, 6);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[6]), 4, 7);
	SendCanMessage((uint8_t*)&(AdcValuesDmaBuffer[7]), 4, 8);

    // test i2c values for fuse 9
	uint8_t address = 0x40;

    uint8_t powerBuffer[2] = { 0 };
    uint8_t currentBuffer[2] = { 0 };
    uint8_t shuntVoltageBuffer[2] = { 0 };
    uint8_t busVoltageBuffer[2] = { 0 };

	HAL_StatusTypeDef statusPower =  HAL_I2C_Mem_Read(&hi2c1, address << 1, 0x03, I2C_MEMADD_SIZE_8BIT, powerBuffer, 2, 500);
	HAL_StatusTypeDef statusCurrent =  HAL_I2C_Mem_Read(&hi2c1, address << 1, 0x04, I2C_MEMADD_SIZE_8BIT, currentBuffer, 2, 500);
	HAL_StatusTypeDef shuntVoltageCurrent =  HAL_I2C_Mem_Read(&hi2c1, address << 1, 0x01, I2C_MEMADD_SIZE_8BIT, shuntVoltageBuffer, 2, 500);
	HAL_StatusTypeDef busVoltageCurrent =  HAL_I2C_Mem_Read(&hi2c1, address << 1, 0x02, I2C_MEMADD_SIZE_8BIT, busVoltageBuffer, 2, 500);

    uint16_t power = 0;
    power = powerBuffer[0] << 8;
    power = power | powerBuffer[1];
    uint16_t current = 0;
    current = currentBuffer[0] << 8;
    current = current | currentBuffer[1];
    uint16_t shuntVoltage = 0;
    shuntVoltage = shuntVoltageBuffer[0] << 8;
    shuntVoltage = shuntVoltage | shuntVoltageBuffer[1];
    uint16_t busVoltage = 0;
    busVoltage = busVoltageBuffer[0] << 8;
    busVoltage = busVoltage | busVoltageBuffer[1];

	SendCanMessage(&(powerBuffer[0]), 2, 15);
	SendCanMessage(&(currentBuffer[0]), 2, 16);
}
