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
#include "ssd1306.h"
#include "max30102_for_stm32_hal.h"
#include "spo2_algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	typedef struct userData_t {
		int hour;
		int min;
		unsigned char am;
		int bpm;
		int spO2;
		int steps;
	} userData_t;

	typedef struct heartData_t {
		uint32_t * irBuffer;
		uint32_t * redBuffer;
		int bufferCounter;
	} heartData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* Definitions for default_task */
osThreadId_t default_taskHandle;
const osThreadAttr_t default_task_attributes = {
  .name = "default_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for wright_to_displ */
osThreadId_t wright_to_displHandle;
const osThreadAttr_t wright_to_displ_attributes = {
  .name = "wright_to_displ",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for read_heart_rate */
osThreadId_t read_heart_rateHandle;
const osThreadAttr_t read_heart_rate_attributes = {
  .name = "read_heart_rate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for read_acceloroma */
osThreadId_t read_acceloromaHandle;
const osThreadAttr_t read_acceloroma_attributes = {
  .name = "read_acceloroma",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for date_and_time_t */
osThreadId_t date_and_time_tHandle;
const osThreadAttr_t date_and_time_t_attributes = {
  .name = "date_and_time_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

//char[] for date and time retrival
char time[30];
char date[30];

struct userData_t userData = {12, 36, 0x1, 0, 00, 0};

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
struct heartData_t userHeartData = {irBuffer, redBuffer, 0};
max30102_t max30102;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
void start_default_task(void *argument);
void start_wright_to_display_task(void *argument);
void start_read_heart_rate_task(void *argument);
void start_read_acceloromater_task(void *argument);
void start_date_and_time_task(void *argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void setupMax30102(max30102_t *max30102);
void max30102_calculate_sample_data(int8_t num_samples);
/*************  Task-Creation-Part-A ******************/

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("BOOTING", Font_7x10 ,White);
  ssd1306_UpdateScreen();

  setupMax30102(&max30102);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of default_task */
  default_taskHandle = osThreadNew(start_default_task, NULL, &default_task_attributes);

  /* creation of wright_to_displ */
  wright_to_displHandle = osThreadNew(start_wright_to_display_task, NULL, &wright_to_displ_attributes);

  /* creation of read_heart_rate */
  read_heart_rateHandle = osThreadNew(start_read_heart_rate_task, NULL, &read_heart_rate_attributes);

  /* creation of read_acceloroma */
  read_acceloromaHandle = osThreadNew(start_read_acceloromater_task, NULL, &read_acceloroma_attributes);

  /* creation of date_and_time_t */
  date_and_time_tHandle = osThreadNew(start_date_and_time_task, NULL, &date_and_time_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /*************  Task-Creation-Part-C *****************/



  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_POS1;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x18;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x11;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x3;
  sAlarm.AlarmTime.Minutes = 0x3;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibration
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_512HZ) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the reference Clock input
  */
  if (HAL_RTCEx_SetRefClock(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : heartRateInterrupt_Pin */
  GPIO_InitStruct.Pin = heartRateInterrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(heartRateInterrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setupMax30102 (max30102_t *max30102)
{
	max30102_init(max30102, &hi2c1);
	max30102_reset(max30102);
	max30102_clear_fifo(max30102);
	// FIFO configurations
	max30102_set_fifo_config(max30102, max30102_smp_ave_2, 1, 17);
	max30102_set_mode(max30102, max30102_spo2);
	// LED configurations
	max30102_set_led_pulse_width(max30102, max30102_pw_16_bit);
	max30102_set_adc_resolution(max30102, max30102_adc_4096);
	max30102_set_sampling_rate(max30102, max30102_sr_800);
	max30102_set_led_current_1(max30102, 10);
	max30102_set_led_current_2(max30102, 10);

	//max30102_set_multi_led_slot_1_2(max30102, max30102_led_red, max30102_led_off);
	//max30102_set_multi_led_slot_3_4(max30102, max30102_led_ir, max30102_led_off);

	// Enter SpO2 mode
	//max30102_set_mode(max30102, max30102_spo2);

	// Enable FIFO_A_FULL interrupt
	max30102_set_a_full(max30102, 1);
	// Enable die temperature measurement
	max30102_set_die_temp_en(max30102, 1);
	// Enable DIE_TEMP_RDY interrupt
	max30102_set_die_temp_rdy(max30102, 1);
	//max30102_set_ppg_rdy(max30102, 1);
	//max30102_set_alc_ovf(max30102, 1);
}

//function based on contents here:
//https://www.instructables.com/Guide-to-Using-MAX30102-Heart-Rate-and-Oxygen-Sens/
void max30102_calculate_sample_data(int8_t num_samples)
{
	/*
	uint8_t MSG[35] = {'\0'};
	sprintf(MSG, "  invalid bpm! %i", (unsigned int)bpm);
	HAL_UART_Transmit(&huart2, MSG, sizeof(MSG),HAL_MAX_DELAY);
	*/
	int8_t msg[35] = {'\0'};

	for(int i = 0; i < num_samples; i++)
	{
//		sprintf(msg, "%i : %i ", (int)max30102._ir_samples[i],(int)max30102._red_samples[i]);
//		HAL_UART_Transmit(&huart2, msg, sizeof(msg),HAL_MAX_DELAY);
		userHeartData.irBuffer[userHeartData.bufferCounter] = max30102._ir_samples[i];
		userHeartData.redBuffer[userHeartData.bufferCounter] = max30102._red_samples[i];
		userHeartData.bufferCounter++;
		if(userHeartData.bufferCounter == BUFFER_SIZE)
		{
			memset(msg, 0, sizeof msg);
			int32_t spo2;
			int8_t spo2V;
			int32_t bpm;
			int8_t bpmV;
			maxim_heart_rate_and_oxygen_saturation(userHeartData.irBuffer, userHeartData.redBuffer, MAX30102_BUFFER_LENGTH-MAX30102_SAMPLING_RATE,
					0, &spo2, &spo2V, &bpm, &bpmV);
	        if(spo2V)
	        {
	        	userData.spO2 = spo2;
	        }
	        else{
	        }

	        if(bpmV)
	        {

	        	userData.bpm = bpm;
	        }
	        else{
	        }
	        userHeartData.bufferCounter = 0;
	        updateScreenTask((void*) &userData);
		}

		memset(msg, 0, sizeof msg);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == heartRateInterrupt_Pin)
	{
		max30102_interrupt_handler(&max30102);

	}
}
/************  Task-Creation-Part-B *****************/


/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_default_task */
/**
  * @brief  Function implementing the default_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_default_task */
void start_default_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_wright_to_display_task */
/**
* @brief Function implementing the wright_to_displ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_wright_to_display_task */
void start_wright_to_display_task(void *argument)
{
  /* USER CODE BEGIN start_wright_to_display_task */
  /* Infinite loop */
  for(;;)
  {

	  ssd1306_Fill(Black);
	  ssd1306_SetCursor(0,0);

	  char temp [4][19];
	  //"     hh-mm PM     ";
	  char *ampm = ((userData.am) ? "AM" : "PM");
	  snprintf(temp[0], 19, "     %02i-%02i %.2s     " ,userData.hour ,userData.min ,ampm);
	  ssd1306_WriteString(temp[0], Font_7x10 ,White);

	  ssd1306_SetCursor(0,20);
	  snprintf(temp[1], 19, "BPM: %i", userData.bpm);
	  ssd1306_WriteString(temp[1], Font_7x10, White);

	  ssd1306_SetCursor(0,30);
	  snprintf(temp[2], 19, "BO2: %i%%", userData.spO2);
	  ssd1306_WriteString(temp[2], Font_7x10, White);


	  ssd1306_SetCursor(0,40);
	  snprintf(temp[3], 19, "Steps: %i", userData.steps);
	  ssd1306_WriteString(temp[3], Font_7x10, White);
	  ssd1306_UpdateScreen();

	  osDelay(1000);

  }
  /* USER CODE END start_wright_to_display_task */
}

/* USER CODE BEGIN Header_start_read_heart_rate_task */
/**
* @brief Function implementing the read_heart_rate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_read_heart_rate_task */
void start_read_heart_rate_task(void *argument)
{
  /* USER CODE BEGIN start_read_heart_rate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100000);
  }
  /* USER CODE END start_read_heart_rate_task */
}

/* USER CODE BEGIN Header_start_read_acceloromater_task */
/**
* @brief Function implementing the read_acceloroma thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_read_acceloromater_task */
void start_read_acceloromater_task(void *argument)
{
  /* USER CODE BEGIN start_read_acceloromater_task */
  /* Infinite loop */
  for(;;)
  {


    osDelay(10000);
  }
  /* USER CODE END start_read_acceloromater_task */
}

/* USER CODE BEGIN Header_start_date_and_time_task */
/**
* @brief Function implementing the date_and_time_t thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_date_and_time_task */
void start_date_and_time_task(void *argument)
{
  /* USER CODE BEGIN start_date_and_time_task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  if(sTime.Hours < 12){
		  userData.am = 1;
		  if(sTime.Hours == 0){
			  userData.hour = 12;
		  } else {
			  userData.hour = sTime.Hours;
		  }

	  } else {
		  userData.am = 0;
		  userData.hour = sTime.Hours - 12;
	  }

	  userData.min = sTime.Minutes;


  }
  /* USER CODE END start_date_and_time_task */
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
