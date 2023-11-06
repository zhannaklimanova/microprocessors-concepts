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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLOCK_SIZE 64 * 1024 // 64 x 2^10
#define WRITE_BUFFER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId buttonTaskHandle;
osThreadId TransmitUARTTasHandle;
osThreadId ReadSensorTaskHandle;
/* USER CODE BEGIN PV */
char bufferUART[98];
uint8_t writeBufferNextIndex = 0;
uint8_t flashBuffer[BLOCK_SIZE]; // stores 1 block which is 64KB

enum ButtonStates
{
	NOT_PRESSED,
	PRESSED
};

enum ProgramStates
{
	TSENSOR,
	MAGNETO,
	PSENSOR,
	GYRO,
	STATISTIC,
	NOT_PRINTING
};

typedef struct
{
	uint32_t numerSample;
	float mean;
	float variance;
} Statistic_float;

typedef struct
{
	uint32_t numerSample;
	int16_t mean;
	int16_t variance;
} Statistic_int16;

typedef struct {
	uint32_t tsensor;
	uint32_t psensor;
	struct MagnetoAddresses {
		uint32_t x;
		uint32_t y;
		uint32_t z;
	}magneto;
	struct GyroAddresses {
		uint32_t x;
		uint32_t y;
		uint32_t z;
	}gyro;
} FlashSensorAddresses;

typedef struct {
	float tsensor[WRITE_BUFFER_SIZE];
	float psensor[WRITE_BUFFER_SIZE];
	struct MagnetoData {
		int16_t x[WRITE_BUFFER_SIZE];
		int16_t y[WRITE_BUFFER_SIZE];
		int16_t z[WRITE_BUFFER_SIZE];
	}magneto;
	struct GyroData {
		float x[WRITE_BUFFER_SIZE];
		float y[WRITE_BUFFER_SIZE];
		float z[WRITE_BUFFER_SIZE];
	}gyro;
} SensorData;

const FlashSensorAddresses FlashBlockStart = {
		.psensor = BLOCK_SIZE * 0,
		.tsensor = BLOCK_SIZE * 1,
		.gyro = {
				.x = BLOCK_SIZE * 2,
				.y = BLOCK_SIZE * 3,
				.z = BLOCK_SIZE * 4,
		},
		.magneto = {
				.x = BLOCK_SIZE * 5,
				.y = BLOCK_SIZE * 6,
				.z = BLOCK_SIZE * 7,
		},
};

FlashSensorAddresses currentAddress = {
		.psensor = BLOCK_SIZE * 0,
		.tsensor = BLOCK_SIZE * 1,
		.gyro = {
				.x = BLOCK_SIZE * 2,
				.y = BLOCK_SIZE * 3,
				.z = BLOCK_SIZE * 4,
		},
		.magneto = {
				.x = BLOCK_SIZE * 5,
				.y = BLOCK_SIZE * 6,
				.z = BLOCK_SIZE * 7,
		},
};

SensorData sensorData;

enum ButtonStates buttonState = NOT_PRESSED;
enum ProgramStates programState = NOT_PRINTING;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_QUADSPI_Init(void);
void StartButtonTask(void const * argument);
void StartTransmitUARTTask(void const * argument);
void StartReadSensorTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void clearFlashBlock(uint32_t blockAddress);
void clearFlashUsed(FlashSensorAddresses sensorAddresses);
void writeBuffToFlash(uint32_t* blockAddress, uint32_t blockStart, uint8_t* writeBuffer, uint32_t size);
void readFlashBlockToBuff(uint32_t* blockAddress, uint32_t blockStart, uint8_t* readBuffer, uint32_t size);
void readSensor(SensorData* sensorData);
void printStat();
void calculateStatInt16(uint32_t *currentSensorAddress, uint32_t flashBlockStart, Statistic_int16 *statistic);
void calculateStatFloat(uint32_t *currentSensorAddress, uint32_t flashBlockStart, Statistic_float *statistic);

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_QUADSPI_Init();
  /* USER CODE BEGIN 2 */
  BSP_TSENSOR_Init();
  BSP_MAGNETO_Init();
  BSP_PSENSOR_Init();
  BSP_GYRO_Init();
  BSP_QSPI_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  clearFlashUsed(FlashBlockStart);

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
  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of TransmitUARTTas */
  osThreadDef(TransmitUARTTas, StartTransmitUARTTask, osPriorityNormal, 0, 256);
  TransmitUARTTasHandle = osThreadCreate(osThread(TransmitUARTTas), NULL);

  /* definition and creation of ReadSensorTask */
  osThreadDef(ReadSensorTask, StartReadSensorTask, osPriorityNormal, 0, 256);
  ReadSensorTaskHandle = osThreadCreate(osThread(ReadSensorTask), NULL);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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
  htim2.Init.Prescaler = 4000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
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
	if (GPIO_Pin==BLUE_BUTTON_Pin)
	{
		buttonState = PRESSED;
	}
}

void clearFlashBlock(uint32_t flashBlockStart)
{
	if (BSP_QSPI_Erase_Block(flashBlockStart) != QSPI_OK)
	{
		Error_Handler();
	}
}

void clearFlashUsed(FlashSensorAddresses sensorAddresses)
{
	clearFlashBlock(sensorAddresses.gyro.x);
	clearFlashBlock(sensorAddresses.gyro.y);
	clearFlashBlock(sensorAddresses.gyro.z);
	clearFlashBlock(sensorAddresses.magneto.x);
	clearFlashBlock(sensorAddresses.magneto.y);
	clearFlashBlock(sensorAddresses.magneto.z);
	clearFlashBlock(sensorAddresses.psensor);
	clearFlashBlock(sensorAddresses.tsensor);
}



void writeBuffToFlash(uint32_t* blockAddress, uint32_t blockStart, uint8_t* writeBuffer, uint32_t size)
{
	if ((*blockAddress - blockStart) >= BLOCK_SIZE)
	{
		clearFlashBlock(blockStart);
		*blockAddress = blockStart;
	}

	if (BSP_QSPI_Write(writeBuffer, *blockAddress, size) != QSPI_OK)
	{
		Error_Handler();
	}
	*blockAddress += size;

}

void readFlashBlockToBuff(uint32_t* blockAddress, uint32_t blockStart, uint8_t* readBuffer, uint32_t size)
{
	uint8_t exitCode = BSP_QSPI_Read(readBuffer, blockStart, size);
	if (exitCode != QSPI_OK){
		Error_Handler();
	}
	*blockAddress = blockStart;
	clearFlashBlock(blockStart);
}

void readSensor(SensorData* sensorData)
{
	if (writeBufferNextIndex == WRITE_BUFFER_SIZE) {
		writeBufferNextIndex = 0;
	}

    sensorData->tsensor[writeBufferNextIndex] = BSP_TSENSOR_ReadTemp();

    int16_t magneto[3];
    BSP_MAGNETO_GetXYZ(magneto);
    sensorData->magneto.x[writeBufferNextIndex] = magneto[0];
    sensorData->magneto.y[writeBufferNextIndex] = magneto[1];
    sensorData->magneto.z[writeBufferNextIndex] = magneto[2];

    sensorData->psensor[writeBufferNextIndex] = BSP_PSENSOR_ReadPressure();

    float gyro[3];
    BSP_GYRO_GetXYZ(gyro);
    sensorData->gyro.x[writeBufferNextIndex] = gyro[0];
    sensorData->gyro.y[writeBufferNextIndex] = gyro[1];
    sensorData->gyro.z[writeBufferNextIndex] = gyro[2];

    writeBufferNextIndex++;
}

void printStat()
{

	Statistic_float statisticF;
	Statistic_int16 statisticI;

	// temp print calc
	calculateStatFloat(&(currentAddress.tsensor), FlashBlockStart.tsensor, &statisticF);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "temp: %ld, %.4f, %.10f\r\n", statisticF.numerSample, statisticF.mean, statisticF.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	// print magne
	calculateStatInt16(&(currentAddress.magneto.x), FlashBlockStart.magneto.x, &statisticI);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "magne.x: %ld, %d, %d\r\n", statisticI.numerSample, statisticI.mean, statisticI.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	calculateStatInt16(&(currentAddress.magneto.y), FlashBlockStart.magneto.y, &statisticI);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "magne.y: %ld, %d, %d\r\n", statisticI.numerSample, statisticI.mean, statisticI.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	calculateStatInt16(&(currentAddress.magneto.z), FlashBlockStart.magneto.z, &statisticI);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "magne.z: %ld, %d, %d\r\n", statisticI.numerSample, statisticI.mean, statisticI.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	// print psensor
	calculateStatFloat(&(currentAddress.psensor), FlashBlockStart.psensor, &statisticF);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "pres: %ld, %.4f, %f\r\n", statisticF.numerSample, statisticF.mean, statisticF.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	// print gyro
	calculateStatFloat(&(currentAddress.gyro.x), FlashBlockStart.gyro.x, &statisticF);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "gyro.x: %ld, %.2f, %.2f\r\n", statisticI.numerSample, statisticF.mean, statisticF.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	calculateStatFloat(&(currentAddress.gyro.y), FlashBlockStart.gyro.y, &statisticF);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "gyro.y: %ld, %.2f, %.2f\r\n", statisticF.numerSample, statisticF.mean, statisticF.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);

	calculateStatFloat(&(currentAddress.gyro.z), FlashBlockStart.gyro.z, &statisticF);
	memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
	sprintf(bufferUART, "gyro.z: %ld, %.2f, %.2f\r\n", statisticI.numerSample, statisticF.mean, statisticF.variance);
	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);


}

void calculateStatInt16(uint32_t *currentSensorAddress, uint32_t flashBlockStart, Statistic_int16 *statistic)
{
	const uint32_t size = (*currentSensorAddress - flashBlockStart);
	statistic->numerSample = size / sizeof(int16_t);
	readFlashBlockToBuff(currentSensorAddress, flashBlockStart, flashBuffer, size);
	arm_mean_q15((q15_t*)flashBuffer, statistic->numerSample, (q15_t*)&(statistic->mean));
	arm_var_q15((q15_t*)flashBuffer, statistic->numerSample, (q15_t*)&(statistic->variance));
}

void calculateStatFloat(uint32_t *currentSensorAddress, uint32_t flashBlockStart, Statistic_float *statistic)
{
	const uint32_t size = (*currentSensorAddress - flashBlockStart);
	statistic->numerSample = size / sizeof(float);
	readFlashBlockToBuff(currentSensorAddress, flashBlockStart, flashBuffer, size);
	arm_mean_f32((float32_t*)flashBuffer, statistic->numerSample, &(statistic->mean));
	arm_var_f32((float32_t*)flashBuffer, statistic->numerSample, &(statistic->variance));
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osDelay(500);
	if(buttonState == PRESSED){
		switch (programState) {
			case NOT_PRINTING:
				programState = TSENSOR;
				break;
			case TSENSOR:
				programState = MAGNETO;
				break;
			case MAGNETO:
				programState = PSENSOR;
				break;
			case PSENSOR:
				programState = GYRO;
				break;
			case GYRO:
				programState = STATISTIC;
				break;
			default:
				break;
		}
	}
	buttonState = NOT_PRESSED;
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTransmitUARTTask */
/**
* @brief Function implementing the TransmitUARTTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitUARTTask */
void StartTransmitUARTTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmitUARTTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    switch (programState) {
		case TSENSOR:
			memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
			sprintf(bufferUART, "temperature: %.2f\r\n", sensorData.tsensor[writeBufferNextIndex-1]);
			break;
		case MAGNETO:
			memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
			sprintf(bufferUART, "magnetic: %d, %d, %d\r\n", sensorData.magneto.x[writeBufferNextIndex-1], sensorData.magneto.y[writeBufferNextIndex-1], sensorData.magneto.z[writeBufferNextIndex-1]);
			break;
		case PSENSOR:
			memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
			sprintf(bufferUART, "pressure: %.2f\r\n", sensorData.psensor[writeBufferNextIndex-1]);
			break;
		case GYRO:
			memset(bufferUART, 0, sizeof(bufferUART)/sizeof(bufferUART[0]));
			sprintf(bufferUART, "gyro: %.2f, %.2f, %.2f\r\n", sensorData.gyro.x[writeBufferNextIndex-1], sensorData.gyro.y[writeBufferNextIndex-1], sensorData.gyro.z[writeBufferNextIndex-1]);
			break;
		default:
			break;
	}
    if (programState != NOT_PRINTING && programState != STATISTIC){
    	HAL_UART_Transmit(&huart1, (uint8_t*)bufferUART, sizeof(bufferUART)/sizeof(bufferUART[0]), 1000);
    }
  }
  /* USER CODE END StartTransmitUARTTask */
}

/* USER CODE BEGIN Header_StartReadSensorTask */
/**
* @brief Function implementing the ReadSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadSensorTask */
void StartReadSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartReadSensorTask */
  /* Infinite loop */
  for(;;)
  {
	osDelay(100); // 10Hz or 0.1s

	if (programState == STATISTIC){
		printStat();
		programState = NOT_PRINTING;
	}
	readSensor(&sensorData);
	if(writeBufferNextIndex == WRITE_BUFFER_SIZE){
		writeBuffToFlash(&(currentAddress.tsensor), FlashBlockStart.tsensor, (uint8_t*)sensorData.tsensor, sizeof(sensorData.tsensor));
		writeBuffToFlash(&(currentAddress.magneto.x), FlashBlockStart.magneto.x, (uint8_t*)sensorData.magneto.x, sizeof(sensorData.magneto.x));
		writeBuffToFlash(&(currentAddress.magneto.y), FlashBlockStart.magneto.y, (uint8_t*)sensorData.magneto.y, sizeof(sensorData.magneto.y));
		writeBuffToFlash(&(currentAddress.magneto.z), FlashBlockStart.magneto.z, (uint8_t*)sensorData.magneto.z, sizeof(sensorData.magneto.z));
		writeBuffToFlash(&(currentAddress.psensor), FlashBlockStart.psensor, (uint8_t*)sensorData.psensor, sizeof(sensorData.psensor));
		writeBuffToFlash(&(currentAddress.gyro.x), FlashBlockStart.gyro.x, (uint8_t*)sensorData.gyro.x, sizeof(sensorData.gyro.x));
		writeBuffToFlash(&(currentAddress.gyro.y), FlashBlockStart.gyro.y, (uint8_t*)sensorData.gyro.y, sizeof(sensorData.gyro.y));
		writeBuffToFlash(&(currentAddress.gyro.z), FlashBlockStart.gyro.z, (uint8_t*)sensorData.gyro.z, sizeof(sensorData.gyro.z));
	}
  }
  /* USER CODE END StartReadSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
