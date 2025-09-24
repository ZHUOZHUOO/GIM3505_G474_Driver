/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define PWM_PREIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))
#define PWM_FREQ 25000
#define CKTIM 170000000
#define DEADTIME_NS 1000
#define PWM_PRSC 0
#define DEADTIME CKTIM/1000000/2*DEADTIME_NS/1000
#define ADC1_VCC_Pin GPIO_PIN_3
#define ADC1_VCC_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define CAL_Pin GPIO_PIN_0
#define CAL_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_1
#define ENABLE_GPIO_Port GPIOB
#define nFault_Pin GPIO_PIN_2
#define nFault_GPIO_Port GPIOB
#define ADC1_NTC_Pin GPIO_PIN_11
#define ADC1_NTC_GPIO_Port GPIOB
#define SW_Bit2_Pin GPIO_PIN_5
#define SW_Bit2_GPIO_Port GPIOB
#define SW_Bit1_Pin GPIO_PIN_6
#define SW_Bit1_GPIO_Port GPIOB
#define SW_Bit0_Pin GPIO_PIN_7
#define SW_Bit0_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
