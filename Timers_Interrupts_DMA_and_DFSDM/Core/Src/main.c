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
// NOTE: that the sampling frequency is 8000Hz and is performed by TIM2 with prescaler=125 and counter=80
#define SIN_NUM_SAMPLES 16 // only affects the generated sin wave so no need to increase this number
#define MAX_DAC_ALIGN_8B 170 // 2/3 of possible dynamic range 2^8 = 256
#define MAX_DAC_ALIGN_12B 2730 // 2/3 of possible dynamic range 2^12 = 4096
#define MAX_AMPLITUDE_OF_SHIFTED_SIN 2 // use in cross product to scale a unitary sin wave to one that has a peak of the max dac
#define AUDIO_BUFFER_SIZE 24000 // 8000samples/sec * 3sec = 24000samples

// Frequency Constants
#define C7 2093
#define B6 1975
#define Ab6 1661
#define G6 1567
#define E6 1318
#define Eb6 1244

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t sinArray[SIN_NUM_SAMPLES];
volatile int32_t audioBuffer[AUDIO_BUFFER_SIZE];
uint32_t systemClkFrequency; // used to store value 80MHz, so we don't call function multiple times

// Polling Frequencies
uint32_t prescalerC7; // polling frequency of C7 or 2093Hz
uint32_t prescalerB6;
uint32_t prescalerAb6;
uint32_t prescalerG6;
uint32_t prescalerE6;
uint32_t prescalerEb6;
uint32_t prescalerMicrophone; // polling frequency of our microphone

enum Notes
{
	noteC7,
	noteB6,
	noteAb6,
	noteG6,
	noteE6,
	noteEb6
};

enum LED
{
	OFF,
	BLINKING,
	ON
};

enum ProgramStates
{
	WAIT_FOR_RECORDING,
	RECORDING,
	POST_PROCESSING,
	WAIT_FOR_PLAYBACK,
	PLAY_NOTES,
	PLAYBACK
};

enum Notes currentNote;
enum LED stateLED = OFF;
enum ProgramStates programState = WAIT_FOR_RECORDING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Generate sinusoidal wave table
  float32_t sinValue = 0;
  // Creating samples for sine wave
  for (uint32_t sample=0; sample<SIN_NUM_SAMPLES; sample++)
  {
	  sinValue = arm_sin_f32(2*PI/SIN_NUM_SAMPLES*sample);
	  sinValue += 1.0; // shift function into positive x (range of sin function is MAX_AMPLITUDE_OF_SHIFTED_SIN = 2)
	  // Using 8 bit so 256 DAC (don't want to go over 2/3 of DAC because it creates clipping so use 170)
	  // We have sin function amplitude of 2 maximum and we want to scale it to be over 170 bit
	  sinValue = sinValue / MAX_AMPLITUDE_OF_SHIFTED_SIN * MAX_DAC_ALIGN_8B; // making the waveform the loudest that it can be on DAC without clipping
	  sinArray[sample] = (uint32_t)sinValue;
  }

  // Calculate prescaler values for each note - how fast we will output samples
  systemClkFrequency = HAL_RCC_GetSysClockFreq();

  // period * system_clock = prescaler * counter
  // system_clock / note_frequency = prescaler * counter
  // Multiply by SIN_NUM_SAMPLES to output at every sample with the frequency, not the full length period
  prescalerC7 = systemClkFrequency / ((htim2.Instance->ARR) * C7 * SIN_NUM_SAMPLES);
  prescalerB6 = systemClkFrequency / ((htim2.Instance->ARR) * B6 * SIN_NUM_SAMPLES);
  prescalerAb6 = systemClkFrequency / ((htim2.Instance->ARR) * Ab6 * SIN_NUM_SAMPLES);
  prescalerG6 = systemClkFrequency / ((htim2.Instance->ARR) * G6 * SIN_NUM_SAMPLES);
  prescalerE6 = systemClkFrequency / ((htim2.Instance->ARR) * E6 * SIN_NUM_SAMPLES);
  prescalerEb6 = systemClkFrequency / ((htim2.Instance->ARR) * Eb6 * SIN_NUM_SAMPLES);
  prescalerMicrophone = htim2.Instance->PSC;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 if (programState==POST_PROCESSING)
	 {
		 for (uint32_t i=0; i<AUDIO_BUFFER_SIZE; i++)
		 {
			audioBuffer[i] = (uint32_t)audioBuffer[i] >> 8; // remove channel information from 24-bit DFSDM output
			audioBuffer[i] = audioBuffer[i] * MAX_DAC_ALIGN_8B; // scale value from microphone to the scale DAC can output
			audioBuffer[i] = (uint32_t)audioBuffer[i] >> 24; // its like dividing by 2^24
		 }
		 programState = WAIT_FOR_PLAYBACK;
		 stateLED = OFF;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */
  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 125;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 40000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  htim4.Init.Prescaler = 40000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @overwrite
 * @brief interrupt service routine for GPIO
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Making sure the interrupt was caused by the PC13 or the BLUE_BUTTON
	if (GPIO_Pin==BLUE_BUTTON_Pin)
	{
		switch(programState)
		{
			case PLAY_NOTES:
				// TIM4 is used to control the note playback
				HAL_TIM_Base_Stop_IT(&htim4);
				HAL_TIM_Base_DeInit(&htim4); // de-initialize the timer so that if its interrupted during note playback, it's reset
			case PLAYBACK:
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
			case WAIT_FOR_RECORDING:
				programState = RECORDING;
				stateLED = BLINKING;
				// DAC_CH1 should be set to Normal so that DAC is stopped when its finished READING the recording buffer
				HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*)audioBuffer, (uint32_t)AUDIO_BUFFER_SIZE);
				break;
			case WAIT_FOR_PLAYBACK:
				programState = PLAY_NOTES;
				stateLED = ON;
				currentNote = noteC7;
				htim2.Instance->PSC = prescalerC7;
				HAL_TIM_Base_Init(&htim4); // bc we de-initialized it during a PLAY_NOTES interrupt, we need to re-initialize
				HAL_TIM_Base_Start_IT(&htim4); // start interrupt
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)sinArray, (uint32_t)SIN_NUM_SAMPLES, DAC_ALIGN_8B_R); // start writing DAC with samples
				break;
			default:
				break;
		}
	}

}

/**
 * @overwrite
 * @brief interrupt service routine for timers
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// SOFTWARE INTERRUPTS SHOULD BE AS SHORT AS POSSIBLE!!!!!!!!!!!!!

	// TIM3 is used to control the LED
	if (htim==&htim3) // ISR for TIM3
	{
		switch(stateLED)
		{
			case OFF:
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
				break;
			case BLINKING:
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // blinks every 0.5 seconds
				break;
			case ON:
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				break;
		}
	} // TIM4 is used to control the note playback
	else if (htim==&htim4)
	{
		switch(currentNote)
		{
			case noteC7:
				currentNote = noteB6;
				htim2.Instance->PSC = prescalerB6;
				break;
			case noteB6:
				currentNote = noteAb6;
				htim2.Instance->PSC = prescalerAb6;
				break;
			case noteAb6:
				currentNote = noteG6;
				htim2.Instance->PSC = prescalerG6;
				break;
			case noteG6:
				currentNote = noteE6;
				htim2.Instance->PSC = prescalerE6;
				break;
			case noteE6:
				currentNote = noteEb6;
				htim2.Instance->PSC = prescalerEb6;
				break;
			case noteEb6:
				htim2.Instance->PSC = prescalerMicrophone;
				HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
				HAL_TIM_Base_Stop_IT(&htim4);
				// DFSDM_FLT0 should be set to Normal so that DFSDM is stopped when its finished WRITING the recording buffer
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)audioBuffer, (uint32_t)AUDIO_BUFFER_SIZE, DAC_ALIGN_8B_R);
				programState = PLAYBACK; // play the recorded sample now and set stateLED = ON;
				break;
		}
	}
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	// SOFTWARE INTERRUPTS SHOULD BE AS SHORT AS POSSIBLE!!!!!!!!!!!!!
	// Playing the recorded sample
	if (programState==PLAYBACK)
	{
		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
		programState = WAIT_FOR_RECORDING; // when finished playback the ISR is done and so go to new state
		stateLED = OFF;
	}
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	// SOFTWARE INTERRUPTS SHOULD BE AS SHORT AS POSSIBLE!!!!!!!!!!!!!
	// ISR for filling buffer during recording
	/*
	 * Instead of doing post-processing in the interrupt we have a flag to do it in main program.
	 * ISR should be as short as possible so as to not stall the other ISR's in the program. By having a
	 * long ISR, you may miss loss some information on some buffer in other hardware components.
	 * Ex. WIFI buffer is also filling but if you are not done yet servicing another interrupt you will
	 * 	   not be able to issue the WIFI interrupt to deal with full buffer so packets trying to fill the WIFI
	 * 	   buffer will be dropped until you clear.
	 */
	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
	programState = POST_PROCESSING; // the stateLED in this state is still BLINKING
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
