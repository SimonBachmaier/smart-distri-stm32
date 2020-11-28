/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FUSE4_FLT_Pin GPIO_PIN_3
#define FUSE4_FLT_GPIO_Port GPIOE
#define FUSE5_FLT_Pin GPIO_PIN_4
#define FUSE5_FLT_GPIO_Port GPIOE
#define FUSE6_FLT_Pin GPIO_PIN_5
#define FUSE6_FLT_GPIO_Port GPIOE
#define FUSE7_FLT_Pin GPIO_PIN_0
#define FUSE7_FLT_GPIO_Port GPIOC
#define FUSE3_FLT_Pin GPIO_PIN_0
#define FUSE3_FLT_GPIO_Port GPIOA
#define FUSE1_IMON_Pin GPIO_PIN_2
#define FUSE1_IMON_GPIO_Port GPIOA
#define FUSE2_IMON_Pin GPIO_PIN_3
#define FUSE2_IMON_GPIO_Port GPIOA
#define FUSE3_IMON_Pin GPIO_PIN_4
#define FUSE3_IMON_GPIO_Port GPIOA
#define FUSE4_IMON_Pin GPIO_PIN_5
#define FUSE4_IMON_GPIO_Port GPIOA
#define FUSE5_IMON_Pin GPIO_PIN_6
#define FUSE5_IMON_GPIO_Port GPIOA
#define FUSE6_IMON_Pin GPIO_PIN_7
#define FUSE6_IMON_GPIO_Port GPIOA
#define FUSE7_IMON_Pin GPIO_PIN_4
#define FUSE7_IMON_GPIO_Port GPIOC
#define FUSE8_IMON_Pin GPIO_PIN_5
#define FUSE8_IMON_GPIO_Port GPIOC
#define FUSE8_FLT_Pin GPIO_PIN_7
#define FUSE8_FLT_GPIO_Port GPIOE
#define FUSE9_FLT_Pin GPIO_PIN_8
#define FUSE9_FLT_GPIO_Port GPIOE
#define FUSE3_SHDN_Pin GPIO_PIN_9
#define FUSE3_SHDN_GPIO_Port GPIOE
#define FUSE4_SHDN_Pin GPIO_PIN_10
#define FUSE4_SHDN_GPIO_Port GPIOE
#define FUSE5_SHDN_Pin GPIO_PIN_11
#define FUSE5_SHDN_GPIO_Port GPIOE
#define FUSE6_SHDN_Pin GPIO_PIN_12
#define FUSE6_SHDN_GPIO_Port GPIOE
#define FUSE7_SHDN_Pin GPIO_PIN_13
#define FUSE7_SHDN_GPIO_Port GPIOE
#define FUSE8_SHDN_Pin GPIO_PIN_14
#define FUSE8_SHDN_GPIO_Port GPIOE
#define FUSE9_SHDN_Pin GPIO_PIN_15
#define FUSE9_SHDN_GPIO_Port GPIOE
#define FUSE1_SHDN_Pin GPIO_PIN_8
#define FUSE1_SHDN_GPIO_Port GPIOC
#define FUSE2_SHDN_Pin GPIO_PIN_9
#define FUSE2_SHDN_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define FUSE1_FLT_Pin GPIO_PIN_0
#define FUSE1_FLT_GPIO_Port GPIOE
#define FUSE2_FLT_Pin GPIO_PIN_1
#define FUSE2_FLT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
