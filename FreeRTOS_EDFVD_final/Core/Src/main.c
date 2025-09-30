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
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Periodi e deadline in ms dei Tasks */
/* LedBlueTask */
#define LED_BLUE_PERIOD_MS    	pdMS_TO_TICKS(5000U)
#define LED_BLUE_DEADLINE_MS  	pdMS_TO_TICKS(5000U)
#define LED_BLUE_WCET_LO_MS  	pdMS_TO_TICKS(250U)
#define LED_BLUE_WCET_HI_MS  	pdMS_TO_TICKS(3000U)

/* LedOrangeTask */
#define LED_ORANGE_PERIOD_MS    pdMS_TO_TICKS(6000U)
#define LED_ORANGE_DEADLINE_MS  pdMS_TO_TICKS(6000U)
#define LED_ORANGE_WCET_LO_MS  	pdMS_TO_TICKS(500U)
#define LED_ORANGE_WCET_HI_MS  	pdMS_TO_TICKS(500U)

/* LedRedTask */
#define LED_RED_PERIOD_MS    	pdMS_TO_TICKS(10000U)
#define LED_RED_DEADLINE_MS  	pdMS_TO_TICKS(10000U)
#define LED_RED_WCET_LO_MS  	pdMS_TO_TICKS(200U)
#define LED_RED_WCET_HI_MS  	pdMS_TO_TICKS(3200U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
static volatile BaseType_t xIdleToggleAllowed = pdTRUE;
static volatile uint8_t usCounterPeriods = 0;
static volatile uint8_t usPeriodsBeforeSwitch = 2;
extern BaseType_t xEDFVD_ModeHI;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
void LedBlueTask(void *argument);
void LedOrangeTask(void *argument);
void LedRedTask(void *argument);	// DO NOT use together with TIM3, they use the same LED

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  //HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
/*
  xPeriodicTaskEDFCreate(LedBlueTask,
					  "LedBlue",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  pdMS_TO_TICKS(LED_BLUE_PERIOD_MS),
					  pdMS_TO_TICKS(LED_BLUE_DEADLINE_MS),
					  NULL);
*/
  xPeriodicTaskEDFVDCreate(LedBlueTask,
		  	  	  	  "LedBlue",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  LED_BLUE_PERIOD_MS,
					  LED_BLUE_DEADLINE_MS,
					  1,	// eCRITICALITY_HI
					  LED_BLUE_WCET_LO_MS,
					  LED_BLUE_WCET_HI_MS,
					  NULL);
/*
  xPeriodicTaskEDFCreate(LedOrangeTask,
					  "LedOrange",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  pdMS_TO_TICKS(LED_ORANGE_PERIOD_MS),
					  pdMS_TO_TICKS(LED_ORANGE_DEADLINE_MS),
					  NULL);
*/
  xPeriodicTaskEDFVDCreate(LedOrangeTask,
		  	  	  	  "LedOrange",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  LED_ORANGE_PERIOD_MS,
					  LED_ORANGE_DEADLINE_MS,
					  0,	// eCRITICALITY_LO
					  LED_ORANGE_WCET_LO_MS,
					  LED_ORANGE_WCET_HI_MS,
					  NULL);
/*
  xPeriodicTaskEDFCreate(LedRedTask,
					  "LedRede",
					  configMINIMAL_STACK_SIZE,
					  NULL,
					  pdMS_TO_TICKS(LED_RED_PERIOD_MS),
					  pdMS_TO_TICKS(LED_RED_DEADLINE_MS),
					  NULL);
*/
  xPeriodicTaskEDFVDCreate(LedRedTask,
  		  	  	  	  "LedRed",
  					  configMINIMAL_STACK_SIZE,
  					  NULL,
  					  LED_RED_PERIOD_MS,
					  LED_RED_DEADLINE_MS,
					  1,	// eCRITICALITY_HI
					  LED_RED_WCET_LO_MS,
					  LED_RED_WCET_HI_MS,
  					  NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  vTaskStartScheduler();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  htim3.Init.Prescaler = 168000 - 1;	// 168 MHz / (167999+1) = 1 kHz tick
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500 - 1;		// 500 ticks a 1 ms = 500 ms
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Idle Hook called at every Idle Task cycle */
void vApplicationIdleHook( void )
{
	/* Toggle the Green LED only if it is the first execution of the idle
	 * task after the execution of another task's job.
	 */
	if(xIdleToggleAllowed == pdTRUE)
	{
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		xIdleToggleAllowed = pdFALSE;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_LedBlueTask */
/**
  * @brief  Function implementing the LedBlueTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedBlueTask */
void LedBlueTask(void *argument)
{
	/* USER CODE BEGIN 5 */
	TickType_t xWCET = LED_BLUE_WCET_LO_MS;
	/* Infinite loop */
	for(;;)
	{
		/* If the previous running task was the idle task, update the variable.
		 * In this way, if the idle task starts running again, it will toggle
		 * the Green LED.
		 */
		if(xIdleToggleAllowed == pdFALSE)
		{
			xIdleToggleAllowed = pdTRUE;
		}

		/* Turn on the Blue LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

		TickType_t xStart = xTaskGetTickCount();

		/* Using the variables usCounterPeriods and usPeriodsBeforeSwitch
		 * simulate the condition for a mode switch.
		 */
		if(usCounterPeriods < usPeriodsBeforeSwitch)
		{
			usCounterPeriods++;
			xWCET = LED_BLUE_WCET_LO_MS;
		}
		else
		{
			xWCET = LED_BLUE_WCET_HI_MS;
		}

		/* Busy wait to simulate the task's job execution time. */
		while ((xTaskGetTickCount() - xStart) < xWCET)
		{
			// LedBlueTask keep working
		}

		/* Turn off the Blue LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);

		/* Put the task on hold until the next job is periodically released. */
		vJobTerminate();
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LedOrangeTask */
/**
  * @brief  Function implementing the LedOrangeTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedOrangeTask */
void LedOrangeTask(void *argument)
{
	/* USER CODE BEGIN 6 */
	TickType_t xWCET = LED_ORANGE_WCET_LO_MS;
	/* Infinite loop */
	for(;;)
	{
		/* If the previous running task was the idle task, update the variable.
		 * In this way, if the idle task starts running again, it will toggle
		 * the Green LED.
		 */
		if(xIdleToggleAllowed == pdFALSE)
		{
			xIdleToggleAllowed = pdTRUE;
		}

		/* Turn on the Orange LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		TickType_t xStart = xTaskGetTickCount();

		/* Busy wait to simulate the task's job execution time. */
		while ((xTaskGetTickCount() - xStart) < xWCET)
		{
			// LedOrangeTask keep working
		}

		/* Turn off the Orange LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		/* Put the task on hold until the next job is periodically released. */
		vJobTerminate();
	}
	/* USER CODE END 6 */
}

/* USER CODE BEGIN Header_LedRedTask */
/**
  * @brief  Function implementing the LedRedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedRedTask */
void LedRedTask(void *argument)
{
	/* USER CODE BEGIN 7 */
	/* Infinite loop */
	for(;;)
	{
/* If the previous running task was the idle task, update the variable.
		 * In this way, if the idle task starts running again, it will toggle
		 * the Green LED.
		 */
		if(xIdleToggleAllowed == pdFALSE)
		{
			xIdleToggleAllowed = pdTRUE;
		}

		/* Turn on the Red LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

		TickType_t xStart = xTaskGetTickCount();
		TickType_t xWCET = (xEDFVD_ModeHI == pdFALSE) ? LED_RED_WCET_LO_MS : LED_RED_WCET_HI_MS;

		/* Busy wait to simulate the task's job execution time. */
		while ((xTaskGetTickCount() - xStart) < xWCET)
		{
			// LedRedTask keep working
		}

		/* Turn off the Red LED. */
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

		/* Put the task on hold until the next job is periodically released. */
		vJobTerminate();
	}
	/* USER CODE END 7 */
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
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 7 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 7 */
}
#endif /* USE_FULL_ASSERT */

