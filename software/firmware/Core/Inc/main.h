/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NRF_IRQ_Pin GPIO_PIN_13
#define NRF_IRQ_GPIO_Port GPIOC
#define FLOW_INT_Pin GPIO_PIN_14
#define FLOW_INT_GPIO_Port GPIOC
#define IMU_INT_Pin GPIO_PIN_15
#define IMU_INT_GPIO_Port GPIOC
#define M1_BMEF_U_Pin GPIO_PIN_0
#define M1_BMEF_U_GPIO_Port GPIOC
#define M1_BEMF_V_Pin GPIO_PIN_1
#define M1_BEMF_V_GPIO_Port GPIOC
#define FLOW_MISO_Pin GPIO_PIN_2
#define FLOW_MISO_GPIO_Port GPIOC
#define FLOW_MOSI_Pin GPIO_PIN_3
#define FLOW_MOSI_GPIO_Port GPIOC
#define SERVO_X1_Pin GPIO_PIN_0
#define SERVO_X1_GPIO_Port GPIOA
#define SERVO_Y1_Pin GPIO_PIN_1
#define SERVO_Y1_GPIO_Port GPIOA
#define SERVO_X2_Pin GPIO_PIN_2
#define SERVO_X2_GPIO_Port GPIOA
#define SERVO_Y2_Pin GPIO_PIN_3
#define SERVO_Y2_GPIO_Port GPIOA
#define M2_BEMF_U_Pin GPIO_PIN_4
#define M2_BEMF_U_GPIO_Port GPIOA
#define M2_BEMF_V_Pin GPIO_PIN_5
#define M2_BEMF_V_GPIO_Port GPIOA
#define M2_BMEF_W_Pin GPIO_PIN_6
#define M2_BMEF_W_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_4
#define VBUS_SENSE_GPIO_Port GPIOC
#define FLOW_CS_Pin GPIO_PIN_5
#define FLOW_CS_GPIO_Port GPIOC
#define M1_BEMF_W_Pin GPIO_PIN_2
#define M1_BEMF_W_GPIO_Port GPIOB
#define FLOW_SCK_Pin GPIO_PIN_10
#define FLOW_SCK_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_15
#define NRF_CE_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_10
#define NRF_SCK_GPIO_Port GPIOC
#define NRF_MISO_Pin GPIO_PIN_11
#define NRF_MISO_GPIO_Port GPIOC
#define NRF_MOSI_Pin GPIO_PIN_12
#define NRF_MOSI_GPIO_Port GPIOC
#define NRF_CSN_Pin GPIO_PIN_2
#define NRF_CSN_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOB
#define ESP_TX_Pin GPIO_PIN_6
#define ESP_TX_GPIO_Port GPIOB
#define ESP_RX_Pin GPIO_PIN_7
#define ESP_RX_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
