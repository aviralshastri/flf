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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

#define KP 0.0f
#define KI 0.0f
#define KD 0.0f


#define IR_CHANNEL_COUNT 16
#define CALIB_SAMPLES    50
#define CENTER_VALUE     2047
#define MIN_GROUP_COUNT  10


volatile uint16_t ir_buffer0[IR_CHANNEL_COUNT];
volatile uint16_t ir_buffer1[IR_CHANNEL_COUNT];
volatile uint16_t *ir_buffers[2] = { ir_buffer0, ir_buffer1 };

volatile uint8_t write_buf_idx = 0;
volatile uint8_t read_buf_idx  = 1;
volatile uint8_t current_ir    = 0;
volatile uint16_t motor_a_speed = 0;
volatile uint16_t motor_b_speed = 0;
volatile uint16_t adc_dma_val;
uint32_t last_button_press_time = 0;

volatile uint16_t last_error =0;
volatile float integral=0;
volatile float error = 0.0f;
volatile float derivative=0;
volatile float correction=0;
volatile int8_t weights[16]={-7,-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7,0};
volatile uint32_t time_diff = 0;
volatile uint32_t prev_time = 0;
volatile uint32_t current_time =0;
volatile uint32_t thresholds[IR_CHANNEL_COUNT];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
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
        }

        setMuxChannel(current_ir);

    }
}

void SendIRValuesUART(void)
{
  char buffer[256];
  int len = 0;

  for (int i = 0; i < IR_CHANNEL_COUNT; i++)
  {
    len += sprintf(&buffer[len], "%u",thresholds[i]);

    if (i < IR_CHANNEL_COUNT - 1)
    {
      len += sprintf(&buffer[len], ",");
    }
  }

  len += sprintf(&buffer[len], "\r\n");

  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}



void MotorControl(uint8_t direction_a, uint8_t value_a, uint8_t direction_b, uint8_t value_b)
{
	//given the flf front is facing the observer
	//motor a is left
	//motor b is right

    if(value_a > 100) value_a = 100;
    if(value_b > 100) value_b = 100;


    if(direction_a)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    }

    if(direction_b)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }

    uint32_t pwm_a = (value_a * 999) / 100;
    uint32_t pwm_b = (value_b * 999) / 100;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_b);
}

int boyer_moore_majority(int *arr, int size) {
    int candidate = -1;
    int count = 0;
    for (int i = 0; i < size; i++) {
        if (count == 0) {
            candidate = arr[i];
            count = 1;
        } else if (arr[i] == candidate) {
            count++;
        } else {
            count--;
        }
    }
    return candidate;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_9)
  {
    uint32_t current_time = HAL_GetTick();

    if (current_time - last_button_press_time >= 200)
    {
      last_button_press_time = current_time;

      SendIRValuesUART();
    }
  }
}



void calibration(void) {

// motor start function

int calib_data[CALIB_SAMPLES][IR_CHANNEL_COUNT];
int i, j;

MotorControl(1,100,0,100);

for (i = 0; i < CALIB_SAMPLES; i++) {
for (j = 0; j < IR_CHANNEL_COUNT; j++) {
calib_data[i][j] = ir_buffers[read_buf_idx][j];
HAL_Delay(20);
}
HAL_Delay(10);
}

MotorControl(1,0,1,0);

for (j = 0; j < IR_CHANNEL_COUNT; j++) {
int black_vals[CALIB_SAMPLES], white_vals[CALIB_SAMPLES];
int black_count = 0, white_count = 0;
int min_val = 4095, max_val = 0;

for (i = 0; i < CALIB_SAMPLES; i++) {
int value = calib_data[i][j];

if (value >= CENTER_VALUE) {
black_vals[black_count++] = value;
} else {
white_vals[white_count++] = value;
}

if (value < min_val) min_val = value;
if (value > max_val) max_val = value;
}

if (black_count >= MIN_GROUP_COUNT && white_count >= MIN_GROUP_COUNT) {
int black_majority = boyer_moore_majority(black_vals, black_count);
int white_majority = boyer_moore_majority(white_vals, white_count);
thresholds[j] = (black_majority + white_majority) / 2;
}
else {
thresholds[j] = (min_val + max_val) / 2;
}
}

// motor stop function
}


void pid_loop(void){

    // Step 1: Calculate line position using weighted average
    int weighted_sum = 0;        // Sum of (sensor_value × weight)
    int total_activation = 0;    // Sum of all active sensors

    // Go through each IR sensor
    for (int i = 0; i < IR_CHANNEL_COUNT; i++){
        // Check if sensor detects black line (value > threshold)
        if (ir_buffers[read_buf_idx][i] > thresholds[i]) {
            // Add this sensor's contribution to line position
            weighted_sum += weights[i];      // Add position weight
            total_activation += 1;           // Count this sensor as active
        }
    }

    // Step 2: Calculate error (where is line compared to center?)
    if (total_activation > 0) {
        // Line is detected - calculate average position
        error = (float)weighted_sum / (float)total_activation;
    } else {
        // No line detected - keep last error (robot may be off track)
        // error stays the same as before
    }

    // Step 3: Calculate time difference for derivative and integral
    current_time = HAL_GetTick();                    // Get current time in milliseconds
    time_diff = current_time - prev_time;           // Time since last calculation

    if (time_diff > 0) {  // Make sure we have valid time difference

        // Step 4: Calculate PID components

        // Proportional: Current error
        float proportional = KP * error;

        // Integral: Sum of all past errors (helps with steady-state error)
        integral += error * (float)time_diff / 1000.0f;  // Convert ms to seconds
        float integral_term = KI * integral;

        // Derivative: Rate of change of error (prevents overshooting)
        derivative = (error - last_error) / ((float)time_diff / 1000.0f);
        float derivative_term = KD * derivative;

        // Step 5: Combine all PID terms
        correction = proportional + integral_term + derivative_term;

        // Step 6: Limit correction to prevent extreme values
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

        // Step 7: Apply correction to motor speeds
        int left_speed = BASE_SPEED - (int)correction;   // If correction is +, slow down left motor
        int right_speed = BASE_SPEED + (int)correction;  // If correction is +, speed up right motor

        // Make sure speeds are within valid range (0-100)
        if (left_speed < 0) left_speed = 0;
        if (left_speed > 100) left_speed = 100;
        if (right_speed < 0) right_speed = 0;
        if (right_speed > 100) right_speed = 100;

        // Step 8: Send commands to motors
        MotorControl(1, left_speed, 1, right_speed);  // Both motors forward with different speeds

        // Step 9: Save values for next iteration
        last_error = error;
        prev_time = current_time;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim->Instance == TIM4)
    {

        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_dma_val, 1);

    }
//    else if(htim->Instance == TIM3)
//        {
//
//            pid_loop();
//
//        }


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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
  setMuxChannel(0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  // Bring STBY high to enable driver
  HAL_GPIO_WritePin(GPIOA, STDBY_Pin, GPIO_PIN_SET);

  HAL_Delay(5000);
  calibration();


  /* USER CODE END 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  htim3.Init.Period = 224-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim4.Init.Period = 12-1;
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

  /*Configure GPIO pin : BUTTON2_Pin */
  GPIO_InitStruct.Pin = BUTTON2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
