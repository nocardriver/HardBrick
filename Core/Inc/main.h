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
#include "stm32u5xx_hal.h"

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
#define KEY_LIGHT_Pin GPIO_PIN_0
#define KEY_LIGHT_GPIO_Port GPIOA
#define KEY_LIGHT_EXTI_IRQn EXTI0_IRQn
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define DIS_DC_Pin GPIO_PIN_4
#define DIS_DC_GPIO_Port GPIOC
#define DIS_RST_Pin GPIO_PIN_0
#define DIS_RST_GPIO_Port GPIOB
#define SDMMC_CD_Pin GPIO_PIN_7
#define SDMMC_CD_GPIO_Port GPIOC
#define SDMMC_CD_EXTI_IRQn EXTI7_IRQn
#define PW_EN_Pin GPIO_PIN_3
#define PW_EN_GPIO_Port GPIOB
#define GPS_NRST_Pin GPIO_PIN_8
#define GPS_NRST_GPIO_Port GPIOB
#define DIS_BACKLIGHT_Pin GPIO_PIN_9
#define DIS_BACKLIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
