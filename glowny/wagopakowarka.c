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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//init jeden na całość

typedef struct {//motor
	int max_speed; //30-70?
	int curr_speed; //możliwe zwolnienie i 0
	int mode; //0-off, 1-on, 2-auto
	long change_side_tm; //czas przerzucenia workownicy
	int side;//lewa czy prawa strona
} Motor;

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

typedef struct {//display
	GPIO_TypeDef* latch_port;
	uint16_t latch_pin;
	GPIO_TypeDef* clk_port;
	uint16_t clk_pin;
	GPIO_TypeDef* dt_port;
	uint16_t dt_pin;
} Display;

typedef struct {//led
	GPIO_TypeDef* port;
	uint16_t pin;
} LED;//3

typedef struct {//button
	GPIO_TypeDef* port;
	uint16_t pin;
	int prev_state;
	int curr_state;
} Button;//8

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_FACT_A -2140
#define CAL_FACT_B -2100

#define MOT_1HIST_LO 265 /*13.25kg*/
#define MOT_1HIST_HI 280 /*14.00kg*/
#define MOT_2HIST_LO 291 /*14.55kg*/
#define MOT_2HIST_HI 306 /*15.30kg*/

#define N_WEIGHTS 7
#define REFRESH_DISP 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
const int interval=7500; //delay na odpięcie worka
//cyfry na 7seg
//0-9,pusty,-
//schemat: 0baf gcde
const int num7seg[12]={/*0*/0x08,/*1*/0x3b,/*2*/0x14,/*3*/0x11,/*4*/0x23,/*5*/0x41,/*6*/0x40,/*7*/0x1b,/*8*/0x00,/*9*/0x01,/* */0x7f,/*-*/0x77};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void init_all(Sensor* sensor, Motor* motor, LED* led, Display* display, Button* button);
int read_sensors(Sensor* sensor);
void tare_scale(Sensor* sensor);
void display_number(Display *display, int *weight);
void read_switches(Button *button);
void analyse_switches(Button *button, Motor *motor, Sensor *sensor, int weight);
void set_motor(Motor* motor, LED *led, int weight);
void ShiftOut(Display *display, int number);

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

	//deklaracja zmiennych podstawowych
	Button button[8];
	LED led[3];
	Motor motor;
	Display display;
	Sensor sensor[2];

	//może jakieś zmienne pomocnicze
	int prev_step_tm;
	int counter[2]={0,0};//średnia z pomiarów i wyświetlacz
	int weight[N_WEIGHTS];
	for(int i=0; i<N_WEIGHTS; ++i){
		weight[i]=0;
	}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  init_all(sensor, &motor, led, &display, button);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  set_motor(&motor, led, 0);
  display_number(&display,weight);
  read_sensors(sensor);
  tare_scale(sensor);
  prev_step_tm = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  while((HAL_GetTick()-prev_step_tm) < 100);//ograniczenie do 100ms/cykl*/
	  weight[counter[0]] = read_sensors(sensor);
	  read_switches(button);
	  analyse_switches(button, &motor, sensor, weight[counter[0]]);
	  set_motor(&motor, led, weight[counter[0]]);
	  if (!counter[1]){
		  display_number(&display,weight);
	  }
	  ++counter[0];
	  if(counter[0] >= N_WEIGHTS){
		  counter[0]=0;
	  }
	  ++counter[1];
	  if(counter[1] >= REFRESH_DISP){
		  counter[1]=0;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  HAL_GPIO_WritePin(ScaleA_clk_GPIO_Port, ScaleA_clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Disp_clk_Pin|Disp_dt_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ScaleB_clk_Pin|Disp_latch_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ScaleA_dt_Pin */
  GPIO_InitStruct.Pin = ScaleA_dt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ScaleA_dt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ScaleA_clk_Pin */
  GPIO_InitStruct.Pin = ScaleA_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ScaleA_clk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Disp_clk_Pin Disp_dt_Pin LED3_Pin */
  GPIO_InitStruct.Pin = Disp_clk_Pin|Disp_dt_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button6_Pin Button1_Pin Button2_Pin Button3_Pin
                           Button5_Pin Button4_Pin SwitchB_Pin */
  GPIO_InitStruct.Pin = Button6_Pin|Button1_Pin|Button2_Pin|Button3_Pin
                          |Button5_Pin|Button4_Pin|SwitchB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SwitchA_Pin ScaleB_dt_Pin */
  GPIO_InitStruct.Pin = SwitchA_Pin|ScaleB_dt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ScaleB_clk_Pin Disp_latch_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = ScaleB_clk_Pin|Disp_latch_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//odczyt sensorów
int read_sensors(Sensor* sensor){
	int odczyt1=0, odczyt2=0;
	int stan1,stan2;
	while((HAL_GPIO_ReadPin(sensor[0].dt_port,sensor[0].dt_pin)) == GPIO_PIN_SET);
	while((HAL_GPIO_ReadPin(sensor[1].dt_port,sensor[1].dt_pin)) == GPIO_PIN_SET);
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

//tarowanie wagi
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

//odczyt stanu przycisków
void read_switches(Button *button){
	for(int i=0; i<8; ++i){
		button[i].prev_state = button[i].curr_state;
		button[i].curr_state = HAL_GPIO_ReadPin(button[i].port, button[i].pin);
	}
}

//ustawienia akcji przycisków
void analyse_switches(Button *button, Motor *motor, Sensor* sensor, int weight){
	/*
	 * button[0] = zwiększ prędkość
	 * button[1] = zmniejsz prędkość
	 * button[2] = silnik on
	 * button[3] = silnik auto
	 * button[4] = silnik off
	 * button[5] = taruj
	 * button[6] = lewy switch (od silnika)
	 * button[7] = prawy switch
	 */

	//przerzucenia na workownicy
	if(motor->side == 0){//aktywna strona lewa
		if((button[7].prev_state == 0) && (button[7].curr_state == 0)){
			motor->change_side_tm = HAL_GetTick();//czas zmiany
			motor->side = 1;//aktywna strona prawa
		}
	}
	else if(motor->side == 1){//aktywna strona prawa
		if((button[6].prev_state == 0) && (button[6].curr_state == 0)){
			motor->change_side_tm = HAL_GetTick();//czas zmiany
			motor->side = 0;//aktywna strona lewa
		}
	}

	//zwiększ lub zmniejsz prędkość
	if((button[0].prev_state == 1) && (button[0].curr_state== 0)){
		motor->max_speed += 2;
		if(motor->max_speed > 70) {
			motor->max_speed = 70;
		}
	}
	if((button[1].prev_state == 1) && (button[1].curr_state== 0)){
		motor->max_speed -= 2;
		if(motor->max_speed < 30) {
			motor->max_speed = 30;
		}
	}

	//tryb pracy silnika: on-auto-off
	if((button[2].prev_state == 1) && (button[2].curr_state== 0)){
		motor->mode = 1;
	}
	if((button[3].prev_state == 1) && (button[3].curr_state== 0)){
		motor->mode = 2;
		if(weight <= MOT_1HIST_HI) {
			motor->curr_speed = motor->max_speed;
		}
		else if((weight > MOT_1HIST_HI) && (weight <= MOT_2HIST_HI)){
			motor->curr_speed = (motor->max_speed)/2+10;
		}
		else if(weight > MOT_2HIST_HI){
			motor->curr_speed = 0;
		}
	}
	if((button[4].prev_state == 1) && (button[4].curr_state== 0)){
		motor->mode = 0;
	}

	//tarowanie wagi = tylko przy wyłączonym silniku
	if((button[5].prev_state == 1) && (button[5].curr_state== 0)){
		if(motor->mode == 0) {
			tare_scale(sensor);
		}
	}
}

//ustaw prędkość silnika
void set_motor(Motor* motor, LED *led, int weight){
	//ustawienia LED na bieżąco
	if(motor->mode == 0){//off
		motor->curr_speed = 0;
		HAL_GPIO_WritePin(led[0].port, led[0].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led[1].port, led[1].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led[2].port, led[2].pin, GPIO_PIN_SET);
	}
	else if(motor->mode == 1){//on
		motor->curr_speed = motor->max_speed;
		HAL_GPIO_WritePin(led[0].port, led[0].pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led[1].port, led[1].pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(led[2].port, led[2].pin, GPIO_PIN_RESET);
	}
	else if(motor->mode == 2){//auto
		if((HAL_GetTick()-(motor->change_side_tm)) <= interval){//odpinamy worek
			motor->curr_speed = motor->max_speed;
		}
		else{//ważenie - 3 prędkości na histerezach
			if(weight <= MOT_1HIST_LO){
				motor->curr_speed = motor->max_speed;
			}
			else if((weight > MOT_1HIST_HI) && (weight <= MOT_2HIST_LO)){
				motor->curr_speed = (motor->max_speed)/2+10;
			}
			else if(weight > MOT_2HIST_HI){
				motor->curr_speed = 0;
			}
		}
		HAL_GPIO_WritePin(led[0].port, led[0].pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led[1].port, led[1].pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led[2].port, led[2].pin, GPIO_PIN_RESET);
	}

	//set PWM
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(motor->curr_speed));
}

//wyświetlacz
void display_number(Display *display, int *number){
	int real_weight = 0;
	for(int i=0; i<N_WEIGHTS; ++i){
		real_weight += number[i];
	}
	real_weight = real_weight/N_WEIGHTS*5;
	if(real_weight <= -1000){//waga < -10.00
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_RESET);
		ShiftOut(display,num7seg[11]);
		ShiftOut(display,num7seg[9]);
		ShiftOut(display,num7seg[9]);
		ShiftOut(display,num7seg[9]);
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_SET);
	}
	else if ((real_weight > -1000) && (real_weight < 0)){//waga -9.95 do -0.05
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_RESET);
		ShiftOut(display,num7seg[11]);
		ShiftOut(display,num7seg[-real_weight/100%10]);
		ShiftOut(display,num7seg[-real_weight/10%10]);
		ShiftOut(display,num7seg[-real_weight%10]);
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_SET);
	}
	else if ((real_weight >= 0) && (real_weight < 1000)){//waga 0.00 do 9.95
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_RESET);
		ShiftOut(display,num7seg[10]);
		ShiftOut(display,num7seg[real_weight/100%10]);
		ShiftOut(display,num7seg[real_weight/10%10]);
		ShiftOut(display,num7seg[real_weight%10]);
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_SET);
	}
	else if ((real_weight >= 1000) && (real_weight < 10000)){//waga 10.00 do 99.95
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_RESET);
		ShiftOut(display,num7seg[real_weight/1000%10]);
		ShiftOut(display,num7seg[real_weight/100%10]);
		ShiftOut(display,num7seg[real_weight/10%10]);
		ShiftOut(display,num7seg[real_weight%10]);
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_SET);
	}
	else if (real_weight >= 10000){//waga > 100.00
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_RESET);
		ShiftOut(display,num7seg[9]);
		ShiftOut(display,num7seg[9]);
		ShiftOut(display,num7seg[9]);
		ShiftOut(display,num7seg[9]);
		HAL_GPIO_WritePin(display->latch_port, display->latch_pin, GPIO_PIN_SET);
	}
}

//pomocniczy shiftout z arduino
void ShiftOut(Display *display, int number){
	//w tej konfiguracji 1 to Q7, 128 to Q0
	int stan;
	for(int i=0; i<8; ++i){
		stan = !!(number & (1<<i));
		HAL_GPIO_WritePin(display->dt_port,display->dt_pin,stan);
		HAL_GPIO_WritePin(display->clk_port,display->clk_pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(display->clk_port,display->clk_pin,GPIO_PIN_RESET);
	}
}

//init całości
void init_all(Sensor* sensor, Motor* motor, LED* led, Display* display, Button* button){
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
	//silnik
	motor->change_side_tm = HAL_GetTick();
	motor->curr_speed = 0;
	motor->max_speed = 50;
	motor->mode = 0;
	//jeszcze strona workownicy
	//ledy
	led[0].port = LED1_GPIO_Port;
	led[0].pin = LED1_Pin;
	led[1].port = LED2_GPIO_Port;
	led[1].pin = LED2_Pin;
	led[2].port = LED3_GPIO_Port;
	led[2].pin = LED3_Pin;
	//przyciski
	button[0].port = Button1_GPIO_Port;
	button[0].pin = Button1_Pin;
	button[1].port = Button2_GPIO_Port;
	button[1].pin = Button2_Pin;
	button[2].port = Button3_GPIO_Port;
	button[2].pin = Button3_Pin;
	button[3].port = Button4_GPIO_Port;
	button[3].pin = Button4_Pin;
	button[4].port = Button5_GPIO_Port;
	button[4].pin = Button5_Pin;
	button[5].port = Button6_GPIO_Port;
	button[5].pin = Button6_Pin;
	button[6].port = SwitchA_GPIO_Port;
	button[6].pin = SwitchA_Pin;
	button[7].port = SwitchB_GPIO_Port;
	button[7].pin = SwitchB_Pin;
	for(int i=0; i<6; ++i){
		button[i].prev_state = 1;
		button[i].curr_state = 1;
	}
	button[6].prev_state = HAL_GPIO_ReadPin(button[6].port, button[6].pin);
	button[6].curr_state = button[6].prev_state;
	button[7].prev_state = HAL_GPIO_ReadPin(button[7].port, button[7].pin);
	button[7].curr_state = button[7].prev_state;
	if(button[6].curr_state == GPIO_PIN_RESET){//wciśnięty
		motor->side=0;
	}
	else{
		motor->side=1;
	}

	//wysw7seg
	display->clk_port = Disp_clk_GPIO_Port;
	display->latch_port = Disp_latch_GPIO_Port;
	display->dt_port = Disp_dt_GPIO_Port;
	display->clk_pin = Disp_clk_Pin;
	display->latch_pin = Disp_latch_Pin;
	display->dt_pin = Disp_dt_Pin;

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
