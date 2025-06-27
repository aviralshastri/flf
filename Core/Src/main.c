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
#include <string.h>
#include <stdio.h>

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define KP 0.0f
#define KI 0.0f
#define KD 0.0f

#define IR_CHANNEL_COUNT 16
volatile uint16_t ir_buffer0[IR_CHANNEL_COUNT];
volatile uint16_t ir_buffer1[IR_CHANNEL_COUNT];
volatile uint16_t *ir_buffers[2] = { ir_buffer0, ir_buffer1 };

volatile uint8_t write_buf_idx = 0;
volatile uint8_t read_buf_idx  = 1;
volatile uint8_t current_ir    = 0;
volatile uint16_t motor_a_speed = 0;
volatile uint16_t motor_b_speed = 0;
volatile uint16_t adc_dma_val;

volatile uint8_t scan_complete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        ir_buffers[write_buf_idx][current_ir] = adc_dma_val;

        current_ir++;

        if (current_ir >= IR_CHANNEL_COUNT) {
            current_ir = 0;
            write_buf_idx ^= 1;
            read_buf_idx ^= 1;
            scan_complete = 1;
        }

        setMuxChannel(current_ir);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM4)
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_dma_val, 1);
    }
}

void setMuxChannel(uint8_t ch)
{
    if(ch >= 16) return;

    switch(ch)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 1:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 2:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 3:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 4:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 5:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 6:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 7:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            break;

        case 8:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 9:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 10:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 11:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 12:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 13:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 14:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        case 15:
            HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            break;

        default:
            break;
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
  memset((void*)ir_buffer0, 0, sizeof(ir_buffer0));
  memset((void*)ir_buffer1, 0, sizeof(ir_buffer1));

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  setMuxChannel(0);
  char line[256];

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Bring STBY high to enable driver
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {

      if(scan_complete)
      {
          scan_complete = 0;

          line[0] = '\0';
          for (uint8_t i = 0; i < IR_CHANNEL_COUNT; i++) {
              char tmp[8];
              sprintf(tmp, "%u", ir_buffers[read_buf_idx][i]);
              strcat(line, tmp);
              if (i < IR_CHANNEL_COUNT - 1) strcat(line, ",");
          }
          strcat(line, "\r\n");
          HAL_UART_Transmit(&huart3, (uint8_t*)line, strlen(line), HAL_MAX_DELAY);
      }

      HAL_Delay(10);

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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Period = 5-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, INA2_Pin|INA1_Pin|STDBY_Pin|INB1_Pin
                          |INB2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S3_Pin|S2_Pin|S1_Pin|S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INA2_Pin INA1_Pin STDBY_Pin INB1_Pin
                           INB2_Pin */
  GPIO_InitStruct.Pin = INA2_Pin|INA1_Pin|STDBY_Pin|INB1_Pin
                          |INB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_Pin S2_Pin S1_Pin S0_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S2_Pin|S1_Pin|S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON2_Pin BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON2_Pin|BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
