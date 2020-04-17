/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define VLCD_Pin GPIO_PIN_3
#define VLCD_GPIO_Port GPIOC
#define JOY_CENTER_Pin GPIO_PIN_0
#define JOY_CENTER_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOA
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_5
#define JOY_DOWN_GPIO_Port GPIOA
#define SEG23_Pin GPIO_PIN_6
#define SEG23_GPIO_Port GPIOA
#define SEG0_Pin GPIO_PIN_7
#define SEG0_GPIO_Port GPIOA
#define SEG22_Pin GPIO_PIN_4
#define SEG22_GPIO_Port GPIOC
#define SEG1_Pin GPIO_PIN_5
#define SEG1_GPIO_Port GPIOC
#define SEG21_Pin GPIO_PIN_0
#define SEG21_GPIO_Port GPIOB
#define SEG2_Pin GPIO_PIN_1
#define SEG2_GPIO_Port GPIOB
#define LD_R_Pin GPIO_PIN_2
#define LD_R_GPIO_Port GPIOB
#define LD_G_Pin GPIO_PIN_8
#define LD_G_GPIO_Port GPIOE
#define QSPI_CLK_Pin GPIO_PIN_10
#define QSPI_CLK_GPIO_Port GPIOE
#define QSPI_CS_Pin GPIO_PIN_11
#define QSPI_CS_GPIO_Port GPIOE
#define QSPI_D0_Pin GPIO_PIN_12
#define QSPI_D0_GPIO_Port GPIOE
#define QSPI_D1_Pin GPIO_PIN_13
#define QSPI_D1_GPIO_Port GPIOE
#define QSPI_D2_Pin GPIO_PIN_14
#define QSPI_D2_GPIO_Port GPIOE
#define QSPI_D3_Pin GPIO_PIN_15
#define QSPI_D3_GPIO_Port GPIOE
#define SEG20_Pin GPIO_PIN_12
#define SEG20_GPIO_Port GPIOB
#define SEG3_Pin GPIO_PIN_13
#define SEG3_GPIO_Port GPIOB
#define SEG19_Pin GPIO_PIN_14
#define SEG19_GPIO_Port GPIOB
#define SEG4_Pin GPIO_PIN_15
#define SEG4_GPIO_Port GPIOB
#define SEG18_Pin GPIO_PIN_8
#define SEG18_GPIO_Port GPIOD
#define SEG5_Pin GPIO_PIN_9
#define SEG5_GPIO_Port GPIOD
#define SEG17_Pin GPIO_PIN_10
#define SEG17_GPIO_Port GPIOD
#define SEG6_Pin GPIO_PIN_11
#define SEG6_GPIO_Port GPIOD
#define SEG16_Pin GPIO_PIN_12
#define SEG16_GPIO_Port GPIOD
#define SEG7_Pin GPIO_PIN_13
#define SEG7_GPIO_Port GPIOD
#define SEG15_Pin GPIO_PIN_14
#define SEG15_GPIO_Port GPIOD
#define SEG8_Pin GPIO_PIN_15
#define SEG8_GPIO_Port GPIOD
#define SEG14_Pin GPIO_PIN_6
#define SEG14_GPIO_Port GPIOC
#define SEG9_Pin GPIO_PIN_7
#define SEG9_GPIO_Port GPIOC
#define SEG13_Pin GPIO_PIN_8
#define SEG13_GPIO_Port GPIOC
#define COM0_Pin GPIO_PIN_8
#define COM0_GPIO_Port GPIOA
#define COM1_Pin GPIO_PIN_9
#define COM1_GPIO_Port GPIOA
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SEG10_Pin GPIO_PIN_15
#define SEG10_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_5
#define USART_TX_GPIO_Port GPIOD
#define USART_RX_Pin GPIO_PIN_6
#define USART_RX_GPIO_Port GPIOD
#define M3V3_REG_ON_Pin GPIO_PIN_3
#define M3V3_REG_ON_GPIO_Port GPIOB
#define SEG11_Pin GPIO_PIN_4
#define SEG11_GPIO_Port GPIOB
#define SEG12_Pin GPIO_PIN_5
#define SEG12_GPIO_Port GPIOB
#define COM3_Pin GPIO_PIN_9
#define COM3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
