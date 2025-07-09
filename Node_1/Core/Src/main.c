/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void CAN1_Tx_1(uint16_t distance_generic_1);
void CAN1_Tx_2(uint16_t distance_generic_2);
void CAN1_Tx_3(uint16_t distance_generic_3);
void CAN1_Tx_4(uint16_t distance_generic_4);
uint16_t distance_calculated_node_1(void);
uint16_t distance_calculated_node_2(void);
uint16_t distance_calculated_node_3(void);
uint16_t distance_calculated_node_4(void);
//uint16_t distance_calculated_node_2(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t icFlag_1 = 0;
uint8_t captureIdx_1 = 0;
uint32_t edge1Time_1 = 0, edge2Time_1 = 0;

uint8_t icFlag_2 = 0;
uint8_t captureIdx_2 = 0;
uint32_t edge1Time_2 = 0, edge2Time_2 = 0;

uint8_t icFlag_3 = 0;
uint8_t captureIdx_3 = 0;
uint32_t edge1Time_3 = 0, edge2Time_3 = 0;

uint8_t icFlag_4 = 0;
uint8_t captureIdx_4 = 0;
uint32_t edge1Time_4 = 0, edge2Time_4 = 0;

const float speedOfSound = 0.0343 / 2;

uint16_t distance_1;
uint16_t distance_2;
uint16_t distance_3;
uint16_t distance_4;
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  CAN1_Tx_1(distance_calculated_node_1());
	  HAL_Delay(1000);
	  CAN1_Tx_2(distance_calculated_node_2());
	  HAL_Delay(1000);
	  CAN1_Tx_3(distance_calculated_node_3());
	  HAL_Delay(1000);
	  CAN1_Tx_4(distance_calculated_node_4());
	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_LOOPBACK;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN1_Tx_1(uint16_t distance_generic_1)
{
	char msg[50];

	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t our_message[2];

	our_message[0] = (uint8_t)(distance_generic_1 >> 8);
	our_message[1] = (uint8_t) distance_generic_1;

	TxHeader.DLC = 2;
	TxHeader.StdId = 0x65D;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,our_message,&TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while( HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));

	sprintf(msg,"Message 1 Transmitted\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

void CAN1_Tx_2(uint16_t distance_generic_2)
{
	char msg[50];

	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t our_message[2];

	our_message[0] = (uint8_t)(distance_generic_2 >> 8);
	our_message[1] = (uint8_t) distance_generic_2;

	TxHeader.DLC = 2;
	TxHeader.StdId = 0x66D;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,our_message,&TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while( HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));

	sprintf(msg,"Message 2 Transmitted\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

void CAN1_Tx_3(uint16_t distance_generic_3)
{
	char msg[50];

	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t our_message[2];

	our_message[0] = (uint8_t)(distance_generic_3 >> 8);
	our_message[1] = (uint8_t) distance_generic_3;

	TxHeader.DLC = 2;
	TxHeader.StdId = 0x67D;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,our_message,&TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while( HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));

	sprintf(msg,"Message 3 Transmitted\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

void CAN1_Tx_4(uint16_t distance_generic_4)
{
	char msg[50];

	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t our_message[2];

	our_message[0] = (uint8_t)(distance_generic_4 >> 8);
	our_message[1] = (uint8_t) distance_generic_4;

	TxHeader.DLC = 2;
	TxHeader.StdId = 0x68D;
	TxHeader.IDE   = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if( HAL_CAN_AddTxMessage(&hcan1,&TxHeader,our_message,&TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}

	while( HAL_CAN_IsTxMessagePending(&hcan1,TxMailbox));

	sprintf(msg,"Message 4 Transmitted\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}

uint16_t distance_calculated_node_1(void)
{
	HAL_TIM_Base_Start(&htim7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	while( ! (TIM7->SR & TIM_SR_UIF) );
	TIM7->SR = 0;
	HAL_TIM_Base_Stop(&htim7);

	HAL_TIM_Base_Start(&htim6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	while( ! (TIM6->SR & TIM_SR_UIF) );
	TIM6->SR = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop(&htim6);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	uint32_t startTick_1 = HAL_GetTick();
	do
	{
		if(icFlag_1) break;
	}while((HAL_GetTick() - startTick_1) < 500);
	icFlag_1 = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);

	if(edge2Time_1 > edge1Time_1)
	{
		distance_1 = (uint16_t)(((edge2Time_1 - edge1Time_1) + 0.0f) * speedOfSound);
	}
	else
	{
		distance_1 = 0;
	}
	return distance_1;
}

uint16_t distance_calculated_node_2(void)
{
	HAL_TIM_Base_Start(&htim7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	while( ! (TIM7->SR & TIM_SR_UIF) );
	TIM7->SR = 0;
	HAL_TIM_Base_Stop(&htim7);

	HAL_TIM_Base_Start(&htim6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	while( ! (TIM6->SR & TIM_SR_UIF) );
	TIM6->SR = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop(&htim6);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	uint32_t startTick_2 = HAL_GetTick();
	do
	{
		if(icFlag_2) break;
	}while((HAL_GetTick() - startTick_2) < 500);
	icFlag_2 = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);

	if(edge2Time_2 > edge1Time_2)
	{
		distance_2 = (uint16_t)(((edge2Time_2 - edge1Time_2) + 0.0f) * speedOfSound);
	}
	else
	{
		distance_2 = 0;
	}
	return distance_2;
}

uint16_t distance_calculated_node_3(void)
{
	HAL_TIM_Base_Start(&htim7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	while( ! (TIM7->SR & TIM_SR_UIF) );
	TIM7->SR = 0;
	HAL_TIM_Base_Stop(&htim7);

	HAL_TIM_Base_Start(&htim6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	while( ! (TIM6->SR & TIM_SR_UIF) );
	TIM6->SR = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop(&htim6);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	uint32_t startTick_3 = HAL_GetTick();
	do
	{
		if(icFlag_3) break;
	}while((HAL_GetTick() - startTick_3) < 500);
	icFlag_3 = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);

	if(edge2Time_3 > edge1Time_3)
	{
		distance_3 = (uint16_t)(((edge2Time_3 - edge1Time_3) + 0.0f) * speedOfSound);
	}
	else
	{
		distance_3 = 0;
	}
	return distance_3;
}

uint16_t distance_calculated_node_4(void)
{
	HAL_TIM_Base_Start(&htim7);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	while( ! (TIM7->SR & TIM_SR_UIF) );
	TIM7->SR = 0;
	HAL_TIM_Base_Stop(&htim7);

	HAL_TIM_Base_Start(&htim6);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	while( ! (TIM6->SR & TIM_SR_UIF) );
	TIM6->SR = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop(&htim6);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
	uint32_t startTick_4 = HAL_GetTick();
	do
	{
		if(icFlag_4) break;
	}while((HAL_GetTick() - startTick_4) < 500);
	icFlag_4 = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_4);

	if(edge2Time_4 > edge1Time_4)
	{
		distance_4 = (uint16_t)(((edge2Time_4 - edge1Time_4) + 0.0f) * speedOfSound);
	}
	else
	{
		distance_4 = 0;
	}
	return distance_4;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			if (captureIdx_1 == 0)
			{
				edge1Time_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				captureIdx_1 = 1;
			}
			else
			{
				edge2Time_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				captureIdx_1 = 0;
				icFlag_1 = 1;
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			if (captureIdx_2 == 0)
			{
				edge1Time_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				captureIdx_2 = 1;
			}
			else
			{
				edge2Time_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
				captureIdx_2 = 0;
				icFlag_2 = 0;
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{
			if (captureIdx_3 == 0)
			{
				edge1Time_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				captureIdx_3 = 1;
			}
			else
			{
				edge2Time_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
				captureIdx_3 = 0;
				icFlag_3 = 0;
			}
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if (captureIdx_4 == 0)
			{
				edge1Time_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				captureIdx_4 = 1;
			}
			else
			{
				edge2Time_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				captureIdx_4 = 0;
				icFlag_4 = 0;
			}
		}
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
