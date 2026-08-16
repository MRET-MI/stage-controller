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

/* ── v1 変更: CVD1_CS PE10→PC0, CVD3_CS PB0→PC1, CVD3_DIR PC6→PE5,
 *            CVD3 PLS→PE6(TIM15_CH2), EN は全軸 N.C.（Enable は SPI 経由）── */

/* Motor Driver 1 (X): CS=PC0, EN=N.C.(SPI Enable), DIR=PA8, PLS=TIM1_CH2(PA9) */
#define CVD1_CS_Pin         GPIO_PIN_0
#define CVD1_CS_GPIO_Port   GPIOC
#define CVD1_DIR_Pin        GPIO_PIN_8
#define CVD1_DIR_GPIO_Port  GPIOA

/* Motor Driver 2 (Y): CS=PE0, EN=N.C.(SPI Enable), DIR=PC8, PLS=TIM3_CH4(PC9) */
#define CVD2_CS_Pin         GPIO_PIN_0
#define CVD2_CS_GPIO_Port   GPIOE
#define CVD2_DIR_Pin        GPIO_PIN_8
#define CVD2_DIR_GPIO_Port  GPIOC

/* Motor Driver 3 (Z): CS=PC1, EN=N.C.(SPI Enable), DIR=PE5, PLS=TIM15_CH2(PE6) */
#define CVD3_CS_Pin         GPIO_PIN_1
#define CVD3_CS_GPIO_Port   GPIOC
#define CVD3_DIR_Pin        GPIO_PIN_5
#define CVD3_DIR_GPIO_Port  GPIOE

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

/* 汎用 GPIO 出力4点（GUI トグルで ON/OFF）。CubeMX: GPIO_Output, Push-Pull, 初期 Low, Low speed */
#define GPO0_Pin            GPIO_PIN_9      /* PE9  */
#define GPO0_GPIO_Port      GPIOE
#define GPO1_Pin            GPIO_PIN_10     /* PE10 */
#define GPO1_GPIO_Port      GPIOE
#define GPO2_Pin            GPIO_PIN_11     /* PE11 */
#define GPO2_GPIO_Port      GPIOE
#define GPO3_Pin            GPIO_PIN_12     /* PE12 */
#define GPO3_GPIO_Port      GPIOE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
