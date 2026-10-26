/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define RC_SW1_Pin GPIO_PIN_0
#define RC_SW1_GPIO_Port GPIOE
#define RC_SW2_Pin GPIO_PIN_7
#define RC_SW2_GPIO_Port GPIOE
#define RC_SW3_Pin GPIO_PIN_8
#define RC_SW3_GPIO_Port GPIOE
#define RC_SW4_Pin GPIO_PIN_10
#define RC_SW4_GPIO_Port GPIOE
#define RC_SW5_Pin GPIO_PIN_12
#define RC_SW5_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOE
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOE
#define OLED_RES_Pin GPIO_PIN_1
#define OLED_RES_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_2
#define OLED_DC_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOE
#define S8_Pin GPIO_PIN_12
#define S8_GPIO_Port GPIOB
#define S7_Pin GPIO_PIN_13
#define S7_GPIO_Port GPIOB
#define S6_Pin GPIO_PIN_14
#define S6_GPIO_Port GPIOB
#define S5_Pin GPIO_PIN_15
#define S5_GPIO_Port GPIOB
#define S4_Pin GPIO_PIN_8
#define S4_GPIO_Port GPIOD
#define S3_Pin GPIO_PIN_9
#define S3_GPIO_Port GPIOD
#define S2_Pin GPIO_PIN_10
#define S2_GPIO_Port GPIOD
#define S1_Pin GPIO_PIN_11
#define S1_GPIO_Port GPIOD
#define CIN2_Pin GPIO_PIN_12
#define CIN2_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_0
#define DIN1_GPIO_Port GPIOD
#define DIN2_Pin GPIO_PIN_1
#define DIN2_GPIO_Port GPIOD
#define EIN1_Pin GPIO_PIN_2
#define EIN1_GPIO_Port GPIOD
#define EIN2_Pin GPIO_PIN_3
#define EIN2_GPIO_Port GPIOD
#define FIN1_Pin GPIO_PIN_4
#define FIN1_GPIO_Port GPIOD
#define FIN2_Pin GPIO_PIN_7
#define FIN2_GPIO_Port GPIOD
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_6
#define BIN2_GPIO_Port GPIOB
#define CIN1_Pin GPIO_PIN_7
#define CIN1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
