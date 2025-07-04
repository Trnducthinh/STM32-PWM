/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "N6_LCD.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
 int dem;
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_12
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT11_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}
char buffer[64];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
char data;
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
 HAL_TIM_Base_Start(&htim1);
 lcd_init();
 N6_LCD_Innit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (USART1->SR & (1 << 5))  // RXNE
			{
					data = USART1->DR;  // Ð?c d? li?u

					// Ð?i cho TXE = 1 (s?n sàng g?i)
					while (!(USART1->SR & (1 << 7))); // TXE

					USART1->DR = data;  // G?i l?i (echo)
			}	

		if(dem==1)
		{
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 1:0%  ");
		lcd_put_cur(1,0);
    lcd_send_string("Khoi dong");
		}
		if(dem==2)
		{
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 2:40% ");
		lcd_put_cur(1,0);
    lcd_send_string("           ");
		}
		if(dem==3)
		{
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 3:60% ");
			lcd_put_cur(1,0);
    lcd_send_string("           ");
		}
		if(dem==4)
		{
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 4:100% ");
			lcd_put_cur(1,0);
    lcd_send_string("           ");
		}
 if(dem==5)
  {
    if(DHT11_Start())
    {
      RHI = DHT11_Read(); // Relative humidity integral
      RHD = DHT11_Read(); // Relative humidity decimal
      TCI = DHT11_Read(); // Celsius integral
      TCD = DHT11_Read(); // Celsius decimal
      SUM = DHT11_Read(); // Check sum
      if (RHI + RHD + TCI + TCD == SUM)
      {
        // Can use RHI and TCI for any purposes if whole number only needed
        tCelsius = (float)TCI + (float)(TCD/10.0);
        tFahrenheit = tCelsius * 9/5 + 32;
        RH = (float)RHI + (float)(RHD/10.0);
        // Can use tCelsius, tFahrenheit and RH for any purposes
      }
      if (TCI < 15) 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
      }
      else if (TCI < 25 ) 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
      }
      else 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
      }
    }
		HAL_Delay(5000);
		lcd_put_cur(0,0);
		sprintf((char*) buffer,"T:%0.1f%cC-H:%0.1f%% ",tCelsius,0XDF,RH);
		lcd_send_string(buffer);
		HAL_Delay(2000);
		if(tCelsius >= 33 && tCelsius < 35.9 )
		{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,100);
			lcd_put_cur(1,0);
    lcd_send_string("      50%     ");
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
			lcd_put_cur(1,0);
    lcd_send_string("      0%     ");
		}
		
  }
	switch(data)
			{
				case '1':
					{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 1:0%  ");
		lcd_put_cur(1,0);
    lcd_send_string("Khoi dong");			
						break;
				case '2':
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,60);
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 2:40% ");
		lcd_put_cur(1,0);
    lcd_send_string("           ");		
						break;
				case '3':
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,80);	
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 3:60% ");
			lcd_put_cur(1,0);
    lcd_send_string("           ");
						break;
				case '4':
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);	
		lcd_put_cur(0,0);
		lcd_send_string("Nut nhan 4:100% ");
			lcd_put_cur(1,0);
    lcd_send_string("           ");
						break;	
			case '5':
		if(DHT11_Start())
    {
      RHI = DHT11_Read(); // Relative humidity integral
      RHD = DHT11_Read(); // Relative humidity decimal
      TCI = DHT11_Read(); // Celsius integral
      TCD = DHT11_Read(); // Celsius decimal
      SUM = DHT11_Read(); // Check sum
      if (RHI + RHD + TCI + TCD == SUM)
      {
        // Can use RHI and TCI for any purposes if whole number only needed
        tCelsius = (float)TCI + (float)(TCD/10.0);
        tFahrenheit = tCelsius * 9/5 + 32;
        RH = (float)RHI + (float)(RHD/10.0);
        // Can use tCelsius, tFahrenheit and RH for any purposes
      }
      if (TCI < 15) 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
      }
      else if (TCI < 25 ) 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
      }
      else 
      {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
      }
    }
		HAL_Delay(5000);
		lcd_put_cur(0,0);
		sprintf((char*) buffer,"T:%0.1f%cC-H:%0.1f%% ",tCelsius,0XDF,RH);
		lcd_send_string(buffer);
		HAL_Delay(2000);
		if(tCelsius >= 25 && tCelsius < 33 )
		{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,40);
			lcd_put_cur(1,0);
    lcd_send_string("      40%     ");
		}
	if(tCelsius >= 33 )
		{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,100);
			lcd_put_cur(1,0);
    lcd_send_string("      100%     ");
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,0);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,20);
			lcd_put_cur(1,0);
    lcd_send_string("      20%     ");
		}
  }
						break;	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BT1_Pin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT3_Pin BT5_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT3_Pin|BT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BT4_Pin */
  GPIO_InitStruct.Pin = BT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
