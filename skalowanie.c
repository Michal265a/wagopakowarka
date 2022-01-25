/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {//sensor
	GPIO_TypeDef* clk_port;
	uint16_t clk_pin;
	GPIO_TypeDef* dt_port;
	uint16_t dt_pin;
	long read;
	long offset;
	int calibration_factor;//na sztywno
	int weight;
} Sensor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_FACT_A -2140
#define CAL_FACT_B -2100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void init(Sensor *sensor);
int read_sensors(Sensor* sensor);
void tare_scale(Sensor* sensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Sensor sensor[2];
  char message[80]="a";
  int weight = 0;
  init(sensor);
  read_sensors(sensor);
  tare_scale(sensor);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  weight = read_sensors(sensor);
	  //sprintf(message,"A: %ld, B: %ld\r\n",sensor[0].read,sensor[1].read);
	  sprintf(message,"A: %ld, B: %ld, Masa: %d\r\n",sensor[0].read,sensor[1].read,weight*5);
      HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message),HAL_MAX_DELAY);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ScaleA_clk_Pin|ScaleB_clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ScaleB_dt_Pin ScaleA_dt_Pin */
  GPIO_InitStruct.Pin = ScaleB_dt_Pin|ScaleA_dt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ScaleA_clk_Pin ScaleB_clk_Pin */
  GPIO_InitStruct.Pin = ScaleA_clk_Pin|ScaleB_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int read_sensors(Sensor* sensor){
	int odczyt1=0, odczyt2=0;
	int stan1,stan2;
	while(((HAL_GPIO_ReadPin(sensor[0].dt_port,sensor[0].dt_pin)) == GPIO_PIN_SET));
	while(((HAL_GPIO_ReadPin(sensor[1].dt_port,sensor[1].dt_pin)) == GPIO_PIN_SET));
	for(int i=0; i<24; ++i){
		HAL_GPIO_WritePin(sensor[0].clk_port,sensor[0].clk_pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(sensor[1].clk_port,sensor[1].clk_pin,GPIO_PIN_SET);
		odczyt1 <<=1;
		odczyt2 <<=1;
		HAL_GPIO_WritePin(sensor[0].clk_port,sensor[0].clk_pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(sensor[1].clk_port,sensor[1].clk_pin,GPIO_PIN_RESET);
		stan1 = HAL_GPIO_ReadPin(sensor[0].dt_port,sensor[0].dt_pin);
		stan2 = HAL_GPIO_ReadPin(sensor[1].dt_port,sensor[1].dt_pin);
		odczyt1 += stan1;
		odczyt2 += stan2;
	}
	HAL_GPIO_WritePin(sensor[0].clk_port,sensor[0].clk_pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(sensor[1].clk_port,sensor[1].clk_pin,GPIO_PIN_SET);
	odczyt1 ^= 0x800000;
	odczyt2 ^= 0x800000;
	HAL_GPIO_WritePin(sensor[0].clk_port,sensor[0].clk_pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(sensor[1].clk_port,sensor[1].clk_pin,GPIO_PIN_RESET);

	sensor[0].read = odczyt1;
	sensor[1].read = odczyt2;
	sensor[0].weight = (sensor[0].read-sensor[0].offset)/sensor[0].calibration_factor;
	sensor[1].weight = (sensor[1].read-sensor[1].offset)/sensor[1].calibration_factor;
	return sensor[0].weight+sensor[1].weight;
}

void tare_scale(Sensor *sensor){
	long offset[2] = {0,0};
	for(int i=0; i<10; ++i){
		read_sensors(sensor);
		offset[0] += sensor[0].read;
		offset[1] += sensor[1].read;
	}
	sensor[0].offset = offset[0]/10;
	sensor[1].offset = offset[1]/10;
}

void init(Sensor *sensor){
	//pierwszy sensor
	sensor[0].calibration_factor = CAL_FACT_A;
	sensor[0].dt_port = ScaleA_dt_GPIO_Port;
	sensor[0].dt_pin = ScaleA_dt_Pin;
	sensor[0].clk_port = ScaleA_clk_GPIO_Port;
	sensor[0].clk_pin = ScaleA_clk_Pin;
	//drugi sensor
	sensor[1].calibration_factor = CAL_FACT_B;
	sensor[1].dt_port = ScaleB_dt_GPIO_Port;
	sensor[1].dt_pin = ScaleB_dt_Pin;
	sensor[1].clk_port = ScaleB_clk_GPIO_Port;
	sensor[1].clk_pin = ScaleB_clk_Pin;
	//oba sensory
	for(int i=0; i<2; ++i){
		sensor[i].read = 0;
		sensor[i].weight = 0;
		sensor[i].offset = 0;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
