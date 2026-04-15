/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef USE_FULL_LL_DRIVER
#define USE_FULL_LL_DRIVER
#endif

// Now pull in the LL bus/GPIO/SPI headers:
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MRF_RESET_Pin GPIO_PIN_0
#define MRF_RESET_GPIO_Port GPIOC
#define SPI2_CS_MRF_Pin GPIO_PIN_3
#define SPI2_CS_MRF_GPIO_Port GPIOC
#define BNO080_INT_Pin GPIO_PIN_1
#define BNO080_INT_GPIO_Port GPIOA
#define BNO080_INT_EXTI_IRQn EXTI1_IRQn
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define RS485_DIR_Pin GPIO_PIN_4
#define RS485_DIR_GPIO_Port GPIOA
#define BNO080_RST_Pin GPIO_PIN_15
#define BNO080_RST_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define GYRO_PS0_Pin GPIO_PIN_11
#define GYRO_PS0_GPIO_Port GPIOC
#define MRF_INT_Pin GPIO_PIN_5
#define MRF_INT_GPIO_Port GPIOB
#define MRF_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
