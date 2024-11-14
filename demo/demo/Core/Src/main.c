/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
osThreadId PIDTaskHandle;
osThreadId LEDTaskHandle;
/* USER CODE BEGIN PV */
int16_t lsp, rsp;

uint16_t sensor_value[3];
uint16_t MinOfMax[3] = {2400};
uint16_t MaxOfMin[3] = {1200};
uint16_t v_compare[3];
uint16_t calib_weight[3] = {0, 400, 800};
uint8_t start_learn = 0;
uint8_t run_enable = 0;
uint8_t ready = 0;
float error;
float uP, uI, uD, u;
float deltaT = 0.001;
float pre_uI, pre_error;


////CONFIG HERE/////
int16_t speed = 600;
float kP = 1.5;
float kD = 0.3;

///////////////////



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void StartPIDTask(void const * argument);
void StartLEDTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t get_sensor_mask(void)
{
	uint8_t temp = 0;
	for(uint8_t i = 0; i < 3; i++)
	{
		temp <<= 1;
		if(sensor_value[i] > v_compare[i])
		{
			temp |= 0x01;
		}
		else
		{
			temp &= 0x06;
		}
	}
	return temp;
}

void run(int16_t left_speed, int16_t right_speed)
{
	if(left_speed > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_speed);
	}
	else if(left_speed < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(left_speed));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	if(right_speed > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_speed);
	}
	else if(right_speed < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, abs(right_speed));
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
}

void PID(float kP, float kD)
{
	error = sensor_value[2] - sensor_value[0];

	uP = kP * error;
	uD = kD * ((error - pre_error)/deltaT);

	u = uP + uD;

	if(u > 1000) u = 1000;
	else if(u < -1000) u = -1000;

	pre_error = error;
//	pre_uI = uI;

	lsp = speed - u;
	rsp = speed + u;

	run(lsp, rsp);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BT1_Pin)
	{
		start_learn = 1;
	}
	if(GPIO_Pin == BT2_Pin)
	{
		run_enable = 1;
	}
}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_ADC_Start_DMA(&hadc1, sensor_value, 3);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of PIDTask */
  osThreadDef(PIDTask, StartPIDTask, osPriorityAboveNormal, 0, 128);
  PIDTaskHandle = osThreadCreate(osThread(PIDTask), NULL);

  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityBelowNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADC_LED_2_Pin|ADC_LED_1_Pin|RED_LED_Pin|GREEN_LED_Pin
                          |BLUE_LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT2_Pin */
  GPIO_InitStruct.Pin = BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_LED_2_Pin ADC_LED_1_Pin RED_LED_Pin GREEN_LED_Pin
                           BLUE_LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = ADC_LED_2_Pin|ADC_LED_1_Pin|RED_LED_Pin|GREEN_LED_Pin
                          |BLUE_LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_LED_3_Pin */
  GPIO_InitStruct.Pin = ADC_LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_LED_3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if(start_learn)
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		osDelay(1000);
		for(int i = 0; i < 500000; i++)
		{
			run(200, -200);
			for(uint8_t j = 0; j < 3; j++)
			{
				if(sensor_value[j] > MaxOfMin[j])
				{
					MaxOfMin[j] = sensor_value[j];
				}
				if(sensor_value[j] < MinOfMax[j])
				{
					MinOfMax[j] = sensor_value[j];
				}
			}
		}
		run(0, 0);
		for(uint8_t i = 0; i < 3; i++)
		{
			v_compare[i] = (MinOfMax[i] + MaxOfMin[i]) / 2 + calib_weight[i];
		}
		start_learn = 0;
	}
	switch(get_sensor_mask())
	{
		case 0x04:			// 100
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 1);
			break;
		case 0x06:			// 110
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 1);
			break;
		case 0x02:			// 010
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 0);
			break;
		case 0x03:			// 011
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 0);
			break;
		case 0x01:			// 001
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 0);
			break;
		case 0x05:			// 101
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 1);
			break;
		case 0x07:			// 111
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 1);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 1);
			ready = 0;
			break;
		case 0x00:			// 000
//			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
//			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
//			HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_1_GPIO_Port, ADC_LED_1_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_2_GPIO_Port, ADC_LED_2_Pin, 0);
			HAL_GPIO_WritePin(ADC_LED_3_GPIO_Port, ADC_LED_3_Pin, 0);
			break;
	}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
  /* Infinite loop */
  for(;;)
  {
	if(run_enable)
	{
		osDelay(3000);
		ready = 1;
		run_enable = 0;
	}
	if(ready)
	{
		if(sensor_value[1] > v_compare[1])
		{
			PID(kP, kD);
		}
	}
	else if(!ready)
	{
		uP = 0;
		uI = 0;
		uD = 0;
		error = 0;
		pre_error = 0;
		pre_uI = 0;
		lsp = 0;
		rsp = 0;
		run(0, 0);
	}
    osDelay(1);
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */
  for(;;)
  {
	if(run_enable || ready || start_learn)
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 1);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 1);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 1);
		vTaskDelay(pdMS_TO_TICKS(200));
	}
	else
	{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, 0);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, 0);
		HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, 0);
	}
    osDelay(1);
  }
  /* USER CODE END StartLEDTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
