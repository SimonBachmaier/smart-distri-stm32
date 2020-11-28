/*
 * smart_distri.h
 *
 *  Created on: Nov 13, 2020
 *      Author: Simon
 */

#ifndef INC_SMART_DISTRI_H_
#define INC_SMART_DISTRI_H_

/**
 * Includes
 */
#include <string.h>
#include "main.h"

/**
 * Defines
 */
#define CAN_MESSAGE_ID_START 0xB0
#define CAN_MESSAGE_ID_FUSE_STATUS 0xA
#define CAN_MESSAGE_ID_RESTART_FUSES 0xB

/**
 * External variable definitions
 */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart4;

/**
 * Variable definitions
 */
uint8_t RxData[8];
uint32_t AdcValues[8];
uint32_t AdcValuesDmaBuffer[8];

/**
 *  Function definitions
 */
void Setup(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN1FilterConfig(void);
void CAN1Start(void);

void SendMsgToUart(char* msgText);

void TryToRestartFuses(void);
void SendCANTestMessage(void);
void SendCanMessage(uint8_t* data, uint32_t dataLengthInBits, uint8_t messageId);
void GetFuseStatus(void);
void ReadAdcValuesFromFuseOneToEight(void);
void SendAdcValues(void);

#endif /* INC_SMART_DISTRI_H_ */
