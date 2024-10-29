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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi4;
#define ecat_spi hspi4

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DBG_1_ON	HAL_GPIO_WritePin(DBG_1_GPIO_Port, DBG_1_Pin, GPIO_PIN_SET)
#define DBG_1_OFF	HAL_GPIO_WritePin(DBG_1_GPIO_Port, DBG_1_Pin, GPIO_PIN_RESET)
#define DBG_2_ON	HAL_GPIO_WritePin(DBG_2_GPIO_Port, DBG_2_Pin, GPIO_PIN_SET)
#define DBG_2_OFF	HAL_GPIO_WritePin(DBG_2_GPIO_Port, DBG_2_Pin, GPIO_PIN_RESET)
#define TOGLLE_RED	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin)
#define TOGLLE_GRN	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin)
#define GRN_ON 		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET)
#define GRN_OFF 	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ecat_spi hspi4
#define ECAT_SPI_CLK_Pin GPIO_PIN_2
#define ECAT_SPI_CLK_GPIO_Port GPIOE
#define ECAT_IRQ_Pin GPIO_PIN_3
#define ECAT_IRQ_GPIO_Port GPIOE
#define ECAT_IRQ_EXTI_IRQn EXTI3_IRQn
#define ECAT_CS_Pin GPIO_PIN_4
#define ECAT_CS_GPIO_Port GPIOE
#define ECAT_SPI_MISO_Pin GPIO_PIN_5
#define ECAT_SPI_MISO_GPIO_Port GPIOE
#define ECAT_SPI_MOSI_Pin GPIO_PIN_6
#define ECAT_SPI_MOSI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define B1_EXTI_IRQn EXTI0_IRQn
#define STLINK_RX_Pin GPIO_PIN_9
#define STLINK_RX_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_10
#define STLINK_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOG
#define LD4_Pin GPIO_PIN_14
#define LD4_GPIO_Port GPIOG
#define DBG_1_Pin GPIO_PIN_3
#define DBG_1_GPIO_Port GPIOB
#define DBG_2_Pin GPIO_PIN_4
#define DBG_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
