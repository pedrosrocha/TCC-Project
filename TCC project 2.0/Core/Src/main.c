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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdio.h>
#include <ClarkeParkTransformation.h>
#include <ADC2VoltageValues.h>
#include <controller.h>

#define PC13_LED0_Pin GPIO_PIN_13
#define PC13_LED0_GPIO_Port GPIOC

#define   DEF_MAX_MSG_LENGTH      128
#define   sine_points          360

#define Kp 120
#define Ki 15000
#define limMin -30000
#define limMax 30000
#define T 0.000040

#define cutOffFrequency 5.0f

#define ADC_voltage_constant 0.0008056640625

#define ADC_BUF_SIZE 4


uint16_t sine_values[] =
                  {2074,2100,2126,2152,2178,2204,2231,2257,2282,2308,2334,2360,2385,2411,2436,2461,2487,2512,
                   2536,2561,2586,2610,2634,2658,2682,2706,2729,2753,2776,2798,2821,2843,2865,2887,2909,2930,
                   2951,2972,2993,3013,3033,3052,3072,3091,3109,3128,3146,3164,3181,3198,3215,3231,3247,3263,
                   3278,3293,3307,3321,3335,3348,3361,3374,3386,3397,3409,3420,3430,3440,3450,3459,3468,3476,
                   3484,3491,3498,3505,3511,3517,3522,3527,3531,3535,3538,3541,3544,3546,3547,3548,3549,3549,
                   3549,3548,3547,3546,3544,3541,3538,3535,3531,3527,3522,3517,3511,3505,3498,3491,3484,3476,
                   3468,3459,3450,3440,3430,3420,3409,3397,3386,3374,3361,3348,3335,3321,3307,3293,3278,3263,
                   3247,3231,3215,3198,3181,3164,3146,3128,3109,3091,3072,3052,3033,3013,2993,2972,2951,2930,
                   2909,2887,2865,2843,2821,2798,2776,2753,2729,2706,2682,2658,2634,2610,2586,2561,2536,2512,
                   2487,2461,2436,2411,2385,2360,2334,2308,2282,2257,2231,2204,2178,2152,2126,2100,2074,2048,
                   2021,1995,1969,1943,1917,1891,1864,1838,1813,1787,1761,1735,1710,1684,1659,1634,1608,1583,
                   1559,1534,1509,1485,1461,1437,1413,1389,1366,1342,1319,1297,1274,1252,1230,1208,1186,1165,
                   1144,1123,1102,1082,1062,1043,1023,1004,986,967,949,931,914,897,880,864,848,832,817,802,788,
                   774,760,747,734,721,709,698,686,675,665,655,645,636,627,619,611,604,597,590,584,578,573,568,
                   564,560,557,554,551,549,548,547,546,546,546,547,548,549,551,554,557,560,564,568,573,578,584,
                   590,597,604,611,619,627,636,645,655,665,675,686,698,709,721,734,747,760,774,788,802,817,832,
                   848,864,880,897,914,931,949,967,986,1004,1023,1043,1062,1082,1102,1123,1144,1165,1186,1208,
                   1230,1252,1274,1297,1319,1342,1366,1389,1413,1437,1461,1485,1509,1534,1559,1583,1608,1634,
                   1659,1684,1710,1735,1761,1787,1813,1838,1864,1891,1917,1943,1969,1995,2021,2048};


typedef struct {
   uint16_t SineA;
   uint16_t SineB;
   uint16_t SineC;
}FucntionGeneratorVoltage;
FucntionGeneratorVoltage MeineBench = {0, 0, 0};


VoltageValues Voltages = {0.0f, 0.0f, 0.0f};
VoltageValues ThreePhasesVoltages = {0.0f, 0.0f, 0.0f};
ClarkeTransformationStruct ClarkVal = {0.0f, 0.0f};


PIController pid = {
		Kp,
		Ki,
		limMin,
		limMax,
		T,
		0,0,0,0
};


Filter filt = {
	cutOffFrequency,
	T,
	0,0,0
};



Filter2ndOrder filter2nd = {
		0,0,0,0,0,0,0,0,0,
		T,cutOffFrequency
};

float filteroutput = 0;


//PLLResponse PLL = {0.0f, 0.0f};

float frequency = 50;
volatile uint8_t PeriodCounter4Bench = 0;
uint8_t  Timer16CyclesCounter;
uint16_t CurrentSample = 0;
char message[DEF_MAX_MSG_LENGTH] = "Place Holder/r/n";

float ADC_offset = 1.65;
uint32_t ratio_numerator = 550;
uint32_t ratio_denominator = 100000;

float error = 0;

uint32_t counter_E = 0;

float Voltage_ADC_offset = 0;

uint32_t adc_buffer[ADC_BUF_SIZE] = {1,2,3,4};

int counter_for_DMA_activation = 0;

// temp
uint32_t current_tick = 0, end_tick = 0, diff_tick = 0, us = 0;
float RealFrequeny = 0;
uint8_t c = 0;


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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_UCPD1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t CounterBenchPeriod(float frequency);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	PIDController_Init(&pid);
	LowPassFilter_init(&filt);
	LowPassFilter2ndOrder_init(&filter2nd);
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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_UCPD1_Init();
  MX_TIM7_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  __HAL_RCC_TIM7_CLK_ENABLE();
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);

  PeriodCounter4Bench = CounterBenchPeriod(frequency);
  __HAL_RCC_TIM7_CLK_ENABLE();

 //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, ADC_BUF_SIZE);

  // Start PWM on Timer 3 Channels 1, 2, and 4
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PA6 (Channel 1)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PA7 (Channel 2)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // PB1 (Channel 4)


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		//used for calcutaing the real frequency
		  RealFrequeny = 1000000.0f/(diff_tick);

		//sprintf(message, "current: %lu \t end: %lu \t diff: %lu \r\n",current_tick, end_tick,  diff_tick);
		//sprintf(message, "Voltage A: %u \t Voltage B: %u \t Voltage C: %u \r\n",MeineBench.SineA, MeineBench.SineB, MeineBench.SineC);

		//sprintf(message, "Volt A: %.2f \t Volt B: %.2f \t Volt C: %.2f \t \r\n", Voltages.VoltageA, Voltages.VoltageB, Voltages.VoltageC);
		sprintf(message, "Volt A: %.2f \t Volt B: %.2f \t Volt C: %.2f \t Volt A: %.2f \t Volt B: %.2f \t Volt C: %.2f \t \r\n", ThreePhasesVoltages.VoltageA, ThreePhasesVoltages.VoltageB, ThreePhasesVoltages.VoltageC, Voltages.VoltageA, Voltages.VoltageB, Voltages.VoltageC);
		//sprintf(message, "time: %d \t Current sample: %u\r\n", msCounter, CurrentSample);

		  PeriodCounter4Bench = CounterBenchPeriod(frequency);


		 //sprintf(message, "%.2f \t %u \t %.2f \t %.2f \t \r\n",  pid.Phase, CurrentSample, pid.Frequency/360, frequency);
		 //sprintf(message, "%.2f\t %.2f \r\n",  pid.Phase, error);
		 //sprintf(message, "Alpha: %.2f \t Beta: %.2f \t \r\n",ClarkVal.Alpha, ClarkVal.Beta);
		 //sprintf(message, "%.3f \t %.3f \t  %.3f \t  %.3f \t %lu \t \r\n", filteroutput, pid.Frequency/360.0f, RealFrequeny , pid.Phase, CurrentSample);

		 //sprintf(message, "%.3f \t %.2f \t  %lu \t  %.2f \t %lu \t \r\n", 1000000.0f/filteroutput, pid.Frequency/360.0f, diff_tick , pid.Phase, CurrentSample);



		  //Voltage_ADC_offset = adc_buffer[0];

		  //sprintf(message, "%lu \t %lu \t %lu \t %lu  \t \r\n", adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3]);

		 CDC_Transmit_FS(message, strlen(message));

		 PeriodCounter4Bench = CounterBenchPeriod(frequency);


		 //HAL_GPIO_TogglePin(PC13_LED0_GPIO_Port, PC13_LED0_Pin);
		 //HAL_Delay(1);

		 //HAL_GPIO_TogglePin(PC13_LED0_GPIO_Port, PC13_LED0_Pin);
		 //HAL_Delay(1);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
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
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4094;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim6.Init.Prescaler = 170-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 39;
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
  htim7.Init.Prescaler = 170-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 85-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PB4   ------> UCPD1_CC2
  PB6   ------> UCPD1_CC1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PC13_LED0_GPIO_Port, PC13_LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13_LED0_Pin */
  GPIO_InitStruct.Pin = PC13_LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PC13_LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13_KEY0_Pin */
  GPIO_InitStruct.Pin = PC13_KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PC13_KEY0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){

	 if(htim->Instance == TIM7){
		   Timer16CyclesCounter++;


		   if(Timer16CyclesCounter > PeriodCounter4Bench-1){
		      CurrentSample++;


		      if(CurrentSample > sine_points-1) {
		    	  CurrentSample = 0;

		    	  end_tick = current_tick;
		    	  current_tick = us; //HAL_GetTick();

		    	  if(end_tick < current_tick){
		    		  diff_tick = current_tick - end_tick;
		    	  }else if(end_tick == current_tick){
		    		  diff_tick = diff_tick;
		    	  }else{
		    		  diff_tick = 0xffffffff - end_tick + current_tick ;
		    	  }

		      }


		      Timer16CyclesCounter = 0;

		      MeineBench.SineA = sine_values[CurrentSample];

		      if(CurrentSample+119 > sine_points-1){
		         MeineBench.SineB = sine_values[CurrentSample - 241];
		      }else{
		         MeineBench.SineB = sine_values[CurrentSample + 119];
		      }

		      if(CurrentSample+239 > sine_points-1){
		         MeineBench.SineC = sine_values[CurrentSample - 121];

		      }else{
		         MeineBench.SineC = sine_values[CurrentSample + 239];
		      }
		   }


		   /*
		   counter_E++;
		   if(counter_E > 50000){
			   counter_E = 0;
			   frequency = frequency+1;
			   if(frequency > 70) frequency = 50;
		   }
		   */



		   counter_for_DMA_activation++;
		   if(counter_for_DMA_activation > 5){
			   counter_for_DMA_activation = 0;
			   HAL_ADC_Start(&hadc1);

		   }

			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MeineBench.SineA);
			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MeineBench.SineB);
			 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MeineBench.SineC);


	 }


	 if(htim->Instance == TIM6){
		 us += 40;

		 Voltages.VoltageA =  (float)adc_buffer[0];
		 Voltages.VoltageB =  (float)adc_buffer[1];
		 Voltages.VoltageC =  (float)adc_buffer[2];

		 /*
		 Voltages.VoltageA =  (float)MeineBench.SineA;
		 Voltages.VoltageB =  (float)MeineBench.SineB;
		 Voltages.VoltageC =  (float)MeineBench.SineC;
		*/

		 ADC_offset = (float)adc_buffer[3] * ADC_voltage_constant - 0.10f;
		 ADC_offset = (float)adc_buffer[3];
		 ThreePhasesVoltages = ADC2RealValues(Voltages, ratio_numerator, ratio_denominator, ADC_offset);

		 ClarkVal = abc2alphabeta(ThreePhasesVoltages.VoltageA , ThreePhasesVoltages.VoltageB , ThreePhasesVoltages.VoltageC );
		 //ClarkVal = abc2alphabeta(Voltages.VoltageA , Voltages.VoltageB , Voltages.VoltageC );

		 error = AlphaBetaCalculation(ClarkVal.Alpha, ClarkVal.Beta, pid.Phase);
		 PIDController_Update(&pid, error);
		 integrator(&pid);

		 //filteroutput = lowPassFilter(pid.Frequency/360.0f, &filt);
		 filteroutput = lowPassFilter2ndOrder(pid.Frequency/360.0f, &filter2nd);


	 }
}


uint8_t CounterBenchPeriod(float frequency){
	float Period = 1000000.0f/frequency;
	int place = 4;
	uint8_t PeriodCounter4 = Period/(sine_points*place);

	return PeriodCounter4;
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
