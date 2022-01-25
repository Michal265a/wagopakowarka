/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define ScaleA_dt_Pin GPIO_PIN_0
#define ScaleA_dt_GPIO_Port GPIOF
#define ScaleA_clk_Pin GPIO_PIN_1
#define ScaleA_clk_GPIO_Port GPIOF
#define Disp_clk_Pin GPIO_PIN_0
#define Disp_clk_GPIO_Port GPIOA
#define Disp_dt_Pin GPIO_PIN_1
#define Disp_dt_GPIO_Port GPIOA
#define Button6_Pin GPIO_PIN_2
#define Button6_GPIO_Port GPIOA
#define Button1_Pin GPIO_PIN_3
#define Button1_GPIO_Port GPIOA
#define Button2_Pin GPIO_PIN_4
#define Button2_GPIO_Port GPIOA
#define Button3_Pin GPIO_PIN_5
#define Button3_GPIO_Port GPIOA
#define Button5_Pin GPIO_PIN_7
#define Button5_GPIO_Port GPIOA
#define SwitchA_Pin GPIO_PIN_0
#define SwitchA_GPIO_Port GPIOB
#define ScaleB_clk_Pin GPIO_PIN_1
#define ScaleB_clk_GPIO_Port GPIOB
#define Button4_Pin GPIO_PIN_9
#define Button4_GPIO_Port GPIOA
#define SwitchB_Pin GPIO_PIN_10
#define SwitchB_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOA
#define Disp_latch_Pin GPIO_PIN_3
#define Disp_latch_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
#define ScaleB_dt_Pin GPIO_PIN_6
#define ScaleB_dt_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
