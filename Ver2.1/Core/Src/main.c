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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"		// standard library = stdlib
#include "params.h"
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void robot_setSpeed(int16_t left_speed, int16_t right_speed)
{
	// left speed process - motor left
	if(left_speed > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_speed);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}
	else if(left_speed < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(left_speed));
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	}

	// right speed process - motor right
	if(right_speed > 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_speed);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
	else if(right_speed < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, abs(right_speed));
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
}

void robot_PIDCalib(void)
{
	err = sensor_value[2] - sensor_value[0];

	uP = kP * err;
	uD = kD * ((err - pre_err) / deltaT);
	pre_err = err;

	u = uP + uD;

	if(u > PID_LIMIT_TOP) u = PID_LIMIT_TOP;
	else if(u < -PID_LIMIT_BOT) u = -PID_LIMIT_BOT;

	left_speed = intial_speed + u;
	right_speed = intial_speed - u;

	robot_setSpeed(left_speed, right_speed);
}

uint8_t sensor_writeLED(void)
{
	uint8_t temp;
	for(uint8_t i = 0 ; i < NUM_OF_LINE_SENSOR; i++)
	{
		temp <<= MASK_001;
		if(sensor_value[i] > v_compare[i])
		{
			HAL_GPIO_WritePin(sensor_led_port[i], sensor_led_pin[i], 1);
			temp |= MASK_001;
		}
		else
		{
			HAL_GPIO_WritePin(sensor_led_port[i], sensor_led_pin[i], 0);
			temp &= MASK_110;
		}
	}
	return temp;
}

void robot_setRGB(uint8_t R, uint8_t G, uint8_t B)
{
	HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, R);
	HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, G);
	HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, B);
}

void robot_beepLong(uint16_t millisec)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	HAL_Delay(millisec);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
}

void robot_setBuzzer(uint16_t millisec, uint8_t numOfBeep)
{
	for(uint8_t i = 0; i < numOfBeep; i++)
	{
		robot_beepLong(millisec);
		HAL_Delay(millisec);
	}
}

void resetBuffer(void)
{
	for(uint8_t i = 0; i < SIZE_TX_DATA; i++)
	{
		*(tx_data + i) = 0;
	}
	for(uint8_t i = 0; i < SIZE_COMMAND; i++)
	{
		*(cmd + i) = 0;
	}
}

void prepareToSend(void)
{
	uint8_t *pByte = NULL;
	pByte = &kP;
	for(uint8_t i = 0; i < 4; i++)
	{
		*(tx_data + i) = pByte[i];
	}
	pByte = &kD;
	for(uint8_t i = 4; i < 8; i++)
	{
		*(tx_data + i) = pByte[i - 4];
	}
	pByte = &intialSpeed;
	for(uint8_t i = 8; i < 10; i++)
	{
		*(tx_data + i) = pByte[i - 8];
	}
	HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), HAL_MAX_DELAY);
	resetBuffer();
}

void assignData(void)
{
	uint8_t *pByte = NULL;
	pByte = &kP;
	for(uint8_t i = 0; i < 4; i++)
	{
		pByte[i] = data[i];
	}
	pByte = &kD;
	for(uint8_t i = 4; i < 8; i++)
	{
		pByte[i-4] = data[i];
	}
	pByte = &intialSpeed;
	for(uint8_t i = 8; i < 10; i++)
	{
		pByte[i-8] = data[i];
	}
	resetBuffer();
}

void robot_readFlash(void)
{
	// Đọc dữ liệu từ flash
	for (uint8_t i = 0; i < SIZE_DATA; i++) {
		data[i] = *(uint8_t *)(FLASH_ADDR_TARGET + i);
	}
	assignData();
}

void robot_writeFlash(void)
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t pageError;
	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitStruct.PageAddress = FLASH_ADDR_TARGET;
	eraseInitStruct.NbPages = 1;
	HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

	for(uint8_t i = 0; i < SIZE_DATA; i+=WORD_DISTANCE_BETWEEN)
	{
		robot_setRGB(1, 1, 1);
		uint32_t data_write = data[i] | (data[i + 1] << 8) | (data[i + 2] << 16) | (data[i + 3] << 24);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_TARGET + i, data_write);
		robot_setRGB(0, 0, 0);
	}

	HAL_FLASH_Lock();
	robot_setRGB(1, 0, 1);
	HAL_UART_Receive_IT(&huart1, cmd, sizeof(cmd));
}

void robot_init(void)
{
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) sensor_value, NUM_OF_ALL_SENSORS);

	robot_readFlash();

	robot_setRGB(1, 0, 0);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(0, 1, 0);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(0, 0, 1);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(1, 1, 0);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(0, 1, 1);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(1, 0, 1);
	HAL_Delay(TIMEBLINK_RGB);
	robot_setRGB(1, 1, 1);
	HAL_Delay(TIMEBLINK_RGB_END);
  	robot_setRGB(0, 0, 0);

  	robot_setBuzzer(DEFAULT_TIME_BEEP, DEFAULT_BEEP_NUMS);

  	HAL_UART_Receive_IT(&huart1, cmd, sizeof(cmd));
}

void button_handle(void)
{
	button_event = (HAL_GPIO_ReadPin(BT1_GPIO_Port, BT1_Pin) << 3) |
				   (HAL_GPIO_ReadPin(BT2_GPIO_Port, BT2_Pin) << 2) |
				   (HAL_GPIO_ReadPin(BT3_GPIO_Port, BT3_Pin) << 1) |
				   (HAL_GPIO_ReadPin(BT4_GPIO_Port, BT4_Pin) << 0);

	switch(button_event)
	{
		// khi button 1 duoc nhan
		case BT1_PRESSED:
			run_case = LEARN_AUTO;
			break;

		// khi button 2 duoc nhan
		case BT2_PRESSED:
			run_case = LEARN_MANUAL;
			break;

		// khi button 3 duoc nhan
		case BT3_PRESSED:
			run_case = RUN_NO_TC;
			break;

		// khi button 4 duoc nhan
		case BT4_PRESSED:
			run_case = RUN_WITH_TC;
			break;

		case ALL_BT_UP:
			run_case = STOP;
			break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart -> Instance == USART1)
	{
		if(cmd[0] == '#' && cmd[1] == '\n')
		{
			robot_setRGB(0, 0, 0);
			resetBuffer();
			HAL_UART_Receive_IT(huart, cmd, sizeof(cmd));
		}
		if(cmd[0] == '@' && cmd[1] == '\n')
		{
			robot_setRGB(1, 0, 1);
			prepareToSend();
			HAL_UART_Receive_IT(huart, cmd, sizeof(cmd));
		}
		if(cmd[0] == '$' && cmd[1] == '\n')
		{
			HAL_UART_AbortReceive_IT(huart);
			robot_setRGB(0, 1, 1);
			HAL_UART_Receive_IT(huart, data, sizeof(data));
			readyToAssign = 1;
		}
		if(cmd[0] == '%' && cmd[1] == '\n')
		{
			robot_writeFlash();
		}
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  robot_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(run_case)
	{
		case LEARN_AUTO:
			robot_setBuzzer(LEARN_AUTO_TIME_BEEP, DEFAULT_BEEP_NUMS);
			for(uint32_t i = 0; i < TIME_LEARN_AUTO; i++)
			{
				robot_setSpeed(SPEED_LEARN_AUTO, -SPEED_LEARN_AUTO);
				for(uint8_t j = 0; j < NUM_OF_LINE_SENSOR; j++)
				{
					if(sensor_value[j] > maxOfMin[j])
					{
						maxOfMin[j] = sensor_value[j];
					}
					if(sensor_value[j] < minOfMax[j])
					{
						minOfMax[j] = sensor_value[j];
					}
					v_compare[j] = ((minOfMax[j] + maxOfMin[j]) / 2) + calib_weight[j];
				}
			}
			robot_setSpeed(0, 0);
			run_case = STOP;
			break;
		case LEARN_MANUAL:
			robot_setBuzzer(LEARN_MANUAL_TIME_BEEP, DEFAULT_BEEP_NUMS);
			for(uint32_t i = 0; i < TIME_LEARN_AUTO; i++)
			{
				for(uint8_t j = 0; j < NUM_OF_LINE_SENSOR; j++)
				{
					if(sensor_value[j] > maxOfMin[j])
					{
						maxOfMin[j] = sensor_value[j];
					}
					if(sensor_value[j] < minOfMax[j])
					{
						minOfMax[j] = sensor_value[j];
					}
					v_compare[j] = ((minOfMax[j] + maxOfMin[j]) / 2) + calib_weight[j];
				}
			}
			run_case = STOP;
			break;
		case RUN_NO_TC:
			if((sensor_mask & MASK_4BIT) == MASK_010)
			{
				robot_beepLong(RUN_NO_TC_TIME_BEEP);
				pid_enable = 1;
			}
			run_case = STOP;
			break;
		case RUN_WITH_TC:
			run_with_sensor = 1;
			break;

		case STOP:
		  	if(readyToAssign)
		  	{
		  		HAL_UART_Receive_IT(&huart1, data, sizeof(data));
		  		HAL_Delay(TIMEOUT_RECEIVING_DATA);
		  		assignData();
		  		HAL_UART_AbortReceive_IT(&huart1);
		  		readyToAssign = 0;
			  	HAL_UART_Receive_IT(&huart1, cmd, sizeof(cmd));
				robot_setRGB(1, 0, 1);
		  	}
		  	break;
	}

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
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin|B_Pin|G_Pin
                          |R_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BT3_Pin */
  GPIO_InitStruct.Pin = BT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT4_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin B_Pin G_Pin
                           R_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|B_Pin|G_Pin
                          |R_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  if (htim->Instance == TIM3) {
	if(pid_enable == 1)
	{
		if(sensor_value[1] > v_compare[1])
		{
			if(intial_speed < intialSpeed)
			{
				intial_speed+=ACCEL_SPEED;
			}
			robot_PIDCalib();
		}
	}
	else
	{
		intial_speed = 0;
		robot_setSpeed(0, 0);
	}
  }
  if (htim->Instance == TIM4) {
	button_handle();
	sensor_mask = sensor_writeLED();
	if((sensor_mask & MASK_4BIT) == MASK_111)
	{
		pid_enable = 0;
	}
	if(run_with_sensor == 1)
	{
		if(sensor_value[3] > TC_DETECT_VALUE)
		{
			robot_setRGB(0, 0, 1);
		}
		else
		{
			robot_setRGB(0, 0, 0);
			pid_enable = 1;
			run_with_sensor = 0;
		}
	}
  }
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
