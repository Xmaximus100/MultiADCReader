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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdint.h>
#include <stddef.h>   // size_t
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include <errno.h>
#include <limits.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUFFER_SIZE 32768
#define ADC_BUFFER_SIZE 32
#define USB_BUFFER_SIZE 64
#define ADC_CH_BUFFER_SIZE 4
#define SAMPLE_SIZE 16
#define CHANNELS 8
#define GPDMA1_REQUEST_DCMI_PSSIRX GPDMA1_REQUEST_DCMI
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_USB_PCD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LTC3_BUSY_Pin GPIO_PIN_14
#define LTC3_BUSY_GPIO_Port GPIOC
#define LTC1_BUSY_Pin GPIO_PIN_15
#define LTC1_BUSY_GPIO_Port GPIOC
#define LTC6_BUSY_Pin GPIO_PIN_0
#define LTC6_BUSY_GPIO_Port GPIOB
#define LTC4_BUSY_Pin GPIO_PIN_2
#define LTC4_BUSY_GPIO_Port GPIOB
#define LTC2_BUSY_Pin GPIO_PIN_14
#define LTC2_BUSY_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define LTC0_BUSY_Pin GPIO_PIN_10
#define LTC0_BUSY_GPIO_Port GPIOC
#define LTC5_BUSY_Pin GPIO_PIN_12
#define LTC5_BUSY_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LTC7_BUSY_Pin GPIO_PIN_5
#define LTC7_BUSY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
