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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
	uint8_t dir;
	uint16_t pwmrun;//通道2数值
	uint16_t pwmturn;//通道3数值
	uint16_t pwmshuiping;//通道4数值
	uint16_t pwmchuizhi;//通道5数值
}SBUS_CH_Struct;

typedef union _BaseSerialData_
{
  unsigned char buffer[sizeof(SBUS_CH_Struct)];
	SBUS_CH_Struct protocal_data;
} BaseSerialData;
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
#define LED_SHOW_Pin GPIO_PIN_13
#define LED_SHOW_GPIO_Port GPIOC
#define SPEED95_Pin GPIO_PIN_12
#define SPEED95_GPIO_Port GPIOB
#define SPEED90_Pin GPIO_PIN_13
#define SPEED90_GPIO_Port GPIOB
#define SPEED85_Pin GPIO_PIN_14
#define SPEED85_GPIO_Port GPIOB
#define SPEED80_Pin GPIO_PIN_15
#define SPEED80_GPIO_Port GPIOB
#define SPEED60_Pin GPIO_PIN_15
#define SPEED60_GPIO_Port GPIOA
#define SPEED65_Pin GPIO_PIN_3
#define SPEED65_GPIO_Port GPIOB
#define SPEED70_Pin GPIO_PIN_4
#define SPEED70_GPIO_Port GPIOB
#define SPEED75_Pin GPIO_PIN_5
#define SPEED75_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
