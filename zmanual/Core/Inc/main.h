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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define gripper_Pin GPIO_PIN_4
#define gripper_GPIO_Port GPIOC
#define door_Pin GPIO_PIN_5
#define door_GPIO_Port GPIOC
#define handEn_Pin GPIO_PIN_14
#define handEn_GPIO_Port GPIOB
#define handPul_Pin GPIO_PIN_9
#define handPul_GPIO_Port GPIOA
#define switchOut_Pin GPIO_PIN_12
#define switchOut_GPIO_Port GPIOC
#define motor1Dir_Pin GPIO_PIN_0
#define motor1Dir_GPIO_Port GPIOD
#define motor2Dir_Pin GPIO_PIN_1
#define motor2Dir_GPIO_Port GPIOD
#define handDir_Pin GPIO_PIN_2
#define handDir_GPIO_Port GPIOD
#define motor3Dir_Pin GPIO_PIN_3
#define motor3Dir_GPIO_Port GPIOD
#define motor4Dir_Pin GPIO_PIN_4
#define motor4Dir_GPIO_Port GPIOD
#define switchIn_Pin GPIO_PIN_8
#define switchIn_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
