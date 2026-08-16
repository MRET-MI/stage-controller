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
#include "stm32h7xx_hal.h"

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

/* USER CODE BEGIN Private defines */

/* Motor Driver 1: CS=PE10, EN=PE2, DIR=PA8, PLS=TIM1CH2 */
#define CVD1_CS_Pin         GPIO_PIN_10
#define CVD1_CS_GPIO_Port   GPIOE
#define CVD1_EN_Pin         GPIO_PIN_2
#define CVD1_EN_GPIO_Port   GPIOE
#define CVD1_DIR_Pin        GPIO_PIN_8
#define CVD1_DIR_GPIO_Port  GPIOA

/* Motor Driver 2: CS=PE0, EN=PE1, DIR=PC8, PLS=TIM8CH4 */
#define CVD2_CS_Pin         GPIO_PIN_0
#define CVD2_CS_GPIO_Port   GPIOE
#define CVD2_EN_Pin         GPIO_PIN_1
#define CVD2_EN_GPIO_Port   GPIOE
#define CVD2_DIR_Pin        GPIO_PIN_8
#define CVD2_DIR_GPIO_Port  GPIOC

/* Motor Driver 3: CS=PB0, EN=PE4, DIR=PC6, PLS=TIM3CH2 */
#define CVD3_CS_Pin         GPIO_PIN_0
#define CVD3_CS_GPIO_Port   GPIOB
#define CVD3_EN_Pin         GPIO_PIN_4
#define CVD3_EN_GPIO_Port   GPIOE
#define CVD3_DIR_Pin        GPIO_PIN_6
#define CVD3_DIR_GPIO_Port  GPIOC

/* Stage sensors (ON=Low, internal pull-up, EXTI both-edge). EXTI line = pin number. */
/* 軸1 (X) */
#define ORG1_Pin            GPIO_PIN_11
#define ORG1_GPIO_Port      GPIOC
#define CCW1_Pin            GPIO_PIN_10
#define CCW1_GPIO_Port      GPIOC
#define CW1_Pin             GPIO_PIN_15
#define CW1_GPIO_Port       GPIOA
/* 軸2 (Y) */
#define ORG2_Pin            GPIO_PIN_1
#define ORG2_GPIO_Port      GPIOD
#define CCW2_Pin            GPIO_PIN_0
#define CCW2_GPIO_Port      GPIOD
#define CW2_Pin             GPIO_PIN_12
#define CW2_GPIO_Port       GPIOC
/* 軸3 (Z) */
#define ORG3_Pin            GPIO_PIN_4
#define ORG3_GPIO_Port      GPIOD
#define CCW3_Pin            GPIO_PIN_3
#define CCW3_GPIO_Port      GPIOD
#define CW3_Pin             GPIO_PIN_2
#define CW3_GPIO_Port       GPIOD

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
