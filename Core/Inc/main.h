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
#include "stm32u5xx_nucleo.h"

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
#define SCD30_SCL_Pin GPIO_PIN_0
#define SCD30_SCL_GPIO_Port GPIOC
#define SCD30_SDA_Pin GPIO_PIN_1
#define SCD30_SDA_GPIO_Port GPIOC
#define Lux_Pin GPIO_PIN_3
#define Lux_GPIO_Port GPIOC
#define OXYGEN_Pin GPIO_PIN_0
#define OXYGEN_GPIO_Port GPIOA
#define TX_DATA_Pin GPIO_PIN_1
#define TX_DATA_GPIO_Port GPIOA
#define ONOFF_capteurs_Pin GPIO_PIN_2
#define ONOFF_capteurs_GPIO_Port GPIOA
#define TX_ONOFF_Pin GPIO_PIN_4
#define TX_ONOFF_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define FLAG_Pin GPIO_PIN_12
#define FLAG_GPIO_Port GPIOF
#define FLAG_EXTI_IRQn EXTI12_IRQn
#define RX_DATA_Pin GPIO_PIN_13
#define RX_DATA_GPIO_Port GPIOF
#define RX_DATA_EXTI_IRQn EXTI13_IRQn
#define AD5668_CLR_Pin GPIO_PIN_10
#define AD5668_CLR_GPIO_Port GPIOB
#define BTN_SEND_TEST_Pin GPIO_PIN_15
#define BTN_SEND_TEST_GPIO_Port GPIOD
#define BTN_SEND_TEST_EXTI_IRQn EXTI15_IRQn
#define LED_ERROR_Pin GPIO_PIN_2
#define LED_ERROR_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define Switch_RF_Pin GPIO_PIN_3
#define Switch_RF_GPIO_Port GPIOB
#define AD5668_SYNC_Pin GPIO_PIN_4
#define AD5668_SYNC_GPIO_Port GPIOB
#define AD5668_LDAC_Pin GPIO_PIN_5
#define AD5668_LDAC_GPIO_Port GPIOB
#define BTN_RECEIVE_TEST_Pin GPIO_PIN_6
#define BTN_RECEIVE_TEST_GPIO_Port GPIOB
#define BTN_RECEIVE_TEST_EXTI_IRQn EXTI6_IRQn
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
