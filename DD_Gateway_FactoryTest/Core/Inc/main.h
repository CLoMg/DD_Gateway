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
#include "stm32l4xx_hal.h"

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
#define SW_Pin GPIO_PIN_13
#define SW_GPIO_Port GPIOC
#define BAT_ADC_Pin GPIO_PIN_0
#define BAT_ADC_GPIO_Port GPIOA
#define Flash_SPI_CS_Pin GPIO_PIN_4
#define Flash_SPI_CS_GPIO_Port GPIOA
#define LDR_ADC_Pin GPIO_PIN_5
#define LDR_ADC_GPIO_Port GPIOA
#define LORA2_RST_Pin GPIO_PIN_1
#define LORA2_RST_GPIO_Port GPIOB
#define LORA2_DIO0_Pin GPIO_PIN_6
#define LORA2_DIO0_GPIO_Port GPIOA
#define RS485EN_Pin GPIO_PIN_2
#define RS485EN_GPIO_Port GPIOB
#define LORA2_SPI_NSS_Pin GPIO_PIN_12
#define LORA2_SPI_NSS_GPIO_Port GPIOB
#define LORA2_SPI_SCK_Pin GPIO_PIN_13
#define LORA2_SPI_SCK_GPIO_Port GPIOB
#define LORA2_SPI_MISO_Pin GPIO_PIN_14
#define LORA2_SPI_MISO_GPIO_Port GPIOB
#define LORA2_SPI_MOSI_Pin GPIO_PIN_15
#define LORA2_SPI_MOSI_GPIO_Port GPIOB
#define ETH_4G_RST_Pin GPIO_PIN_8
#define ETH_4G_RST_GPIO_Port GPIOA
#define _4G_WAKE_IN_Pin GPIO_PIN_11
#define _4G_WAKE_IN_GPIO_Port GPIOA
#define _4G_POWER_EN_Pin GPIO_PIN_12
#define _4G_POWER_EN_GPIO_Port GPIOA
#define LORA1_SPI_NSS_Pin GPIO_PIN_15
#define LORA1_SPI_NSS_GPIO_Port GPIOA
#define LORA1_SPI_SCK_Pin GPIO_PIN_3
#define LORA1_SPI_SCK_GPIO_Port GPIOB
#define LORA1_SPI_MISO_Pin GPIO_PIN_4
#define LORA1_SPI_MISO_GPIO_Port GPIOB
#define LORA1_SPI_MOSI_Pin GPIO_PIN_5
#define LORA1_SPI_MOSI_GPIO_Port GPIOB
#define LORA1_RST_Pin GPIO_PIN_6
#define LORA1_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
