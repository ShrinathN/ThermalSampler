/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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
#define RADIO_SW_Pin GPIO_PIN_1
#define RADIO_SW_GPIO_Port GPIOA
#define FS1000A_DATA_Pin GPIO_PIN_2
#define FS1000A_DATA_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_5
#define LED0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define NRF_CS_Pin GPIO_PIN_0
#define NRF_CS_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_1
#define NRF_CE_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_2
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define NTC_THERMISTOR_Pin GPIO_PIN_12
#define NTC_THERMISTOR_GPIO_Port GPIOB
#define RFM95W_IRQ_Pin GPIO_PIN_13
#define RFM95W_IRQ_GPIO_Port GPIOB
#define RFM95W_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define RFM95W_CS_Pin GPIO_PIN_14
#define RFM95W_CS_GPIO_Port GPIOB
#define RFM95W_RESET_Pin GPIO_PIN_15
#define RFM95W_RESET_GPIO_Port GPIOB
#define PROG_EN_Pin GPIO_PIN_12
#define PROG_EN_GPIO_Port GPIOA
#define SENSOR_SW_Pin GPIO_PIN_9
#define SENSOR_SW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
