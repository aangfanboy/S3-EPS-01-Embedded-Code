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
#include "stm32f4xx_hal.h"

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
#define EN_F_6_Pin GPIO_PIN_3
#define EN_F_6_GPIO_Port GPIOE
#define EN_F_5_Pin GPIO_PIN_4
#define EN_F_5_GPIO_Port GPIOE
#define EN_B_4_Pin GPIO_PIN_5
#define EN_B_4_GPIO_Port GPIOE
#define EN_B_5_Pin GPIO_PIN_6
#define EN_B_5_GPIO_Port GPIOE
#define I_READ_F_3_Pin GPIO_PIN_0
#define I_READ_F_3_GPIO_Port GPIOC
#define I_READ_F_5_Pin GPIO_PIN_1
#define I_READ_F_5_GPIO_Port GPIOC
#define V_READ_B3_Pin GPIO_PIN_2
#define V_READ_B3_GPIO_Port GPIOC
#define I_READ_F_6_Pin GPIO_PIN_3
#define I_READ_F_6_GPIO_Port GPIOC
#define I_READ_F_7_Pin GPIO_PIN_0
#define I_READ_F_7_GPIO_Port GPIOA
#define I_READ_F_8_Pin GPIO_PIN_1
#define I_READ_F_8_GPIO_Port GPIOA
#define V_READ_B4_Pin GPIO_PIN_2
#define V_READ_B4_GPIO_Port GPIOA
#define V_READ_B5_Pin GPIO_PIN_3
#define V_READ_B5_GPIO_Port GPIOA
#define V_READ_B1_Pin GPIO_PIN_4
#define V_READ_B1_GPIO_Port GPIOA
#define I_READ_F_4_Pin GPIO_PIN_5
#define I_READ_F_4_GPIO_Port GPIOA
#define I_READ_F_9_Pin GPIO_PIN_6
#define I_READ_F_9_GPIO_Port GPIOA
#define V_READ_B2_Pin GPIO_PIN_7
#define V_READ_B2_GPIO_Port GPIOA
#define I_READ_F_10_Pin GPIO_PIN_4
#define I_READ_F_10_GPIO_Port GPIOC
#define I_READ_F_1_Pin GPIO_PIN_0
#define I_READ_F_1_GPIO_Port GPIOB
#define I_READ_F_2_Pin GPIO_PIN_1
#define I_READ_F_2_GPIO_Port GPIOB
#define EN_F_9_Pin GPIO_PIN_8
#define EN_F_9_GPIO_Port GPIOE
#define PGOD_F_9_Pin GPIO_PIN_11
#define PGOD_F_9_GPIO_Port GPIOE
#define PGOD_F_10_Pin GPIO_PIN_12
#define PGOD_F_10_GPIO_Port GPIOE
#define EN_F_10_Pin GPIO_PIN_13
#define EN_F_10_GPIO_Port GPIOE
#define EN_B_2_Pin GPIO_PIN_15
#define EN_B_2_GPIO_Port GPIOE
#define KILL_SWITCH_Pin GPIO_PIN_7
#define KILL_SWITCH_GPIO_Port GPIOC
#define KILL_SWITCH_EXTI_IRQn EXTI9_5_IRQn
#define EN_F_2_Pin GPIO_PIN_8
#define EN_F_2_GPIO_Port GPIOC
#define PGOD_F_2_Pin GPIO_PIN_9
#define PGOD_F_2_GPIO_Port GPIOC
#define EN_F_1_Pin GPIO_PIN_8
#define EN_F_1_GPIO_Port GPIOA
#define PGOD_F_1_Pin GPIO_PIN_9
#define PGOD_F_1_GPIO_Port GPIOA
#define EN_B_3_Pin GPIO_PIN_10
#define EN_B_3_GPIO_Port GPIOA
#define EN_F_3_Pin GPIO_PIN_0
#define EN_F_3_GPIO_Port GPIOD
#define EN_F_4_Pin GPIO_PIN_1
#define EN_F_4_GPIO_Port GPIOD
#define PGOD_F_3_Pin GPIO_PIN_2
#define PGOD_F_3_GPIO_Port GPIOD
#define PGOD_F_4_Pin GPIO_PIN_3
#define PGOD_F_4_GPIO_Port GPIOD
#define PGOD_F_5_Pin GPIO_PIN_4
#define PGOD_F_5_GPIO_Port GPIOD
#define PGOD_F_6_Pin GPIO_PIN_5
#define PGOD_F_6_GPIO_Port GPIOD
#define PGOD_F_7_Pin GPIO_PIN_6
#define PGOD_F_7_GPIO_Port GPIOD
#define PGOD_F_8_Pin GPIO_PIN_7
#define PGOD_F_8_GPIO_Port GPIOD
#define EN_F_7_Pin GPIO_PIN_3
#define EN_F_7_GPIO_Port GPIOB
#define EN_F_8_Pin GPIO_PIN_4
#define EN_F_8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
