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
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include "BME280_STM32.h"


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
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
float Temperature, Pressure, Humidity;

uint8_t counterMode = 0;
uint8_t isAlarm = 0;
uint8_t alarmIsSet = 0;

uint8_t activeScreen = 0;
uint8_t screenTime = 0;
uint8_t screenWeather = 1;
uint8_t screenTimer = 2;
uint8_t screenMenu = 3;
uint8_t screenSetTime = 4;

uint8_t activeMenuOption = 0;
uint8_t menuSetTime = 0;
uint8_t menuSetDate = 1;

uint8_t activeSetTimeOption = 0;
//options names
uint8_t setTimeSecondsOption = 0;
uint8_t setTimeMinutesOption = 1;
uint8_t setTimeHoursOption = 2;
//values
uint8_t setTimeHours = 0;
uint8_t setTimeMinutes = 0;
uint8_t setTimeSeconds = 0;

uint8_t counter = 0;
uint8_t prevCounter = 0;
uint8_t encoderPrevState = 0;

uint8_t alarmIterations = 0;

RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
RTC_AlarmTypeDef sAlarm = {0};
RTC_DateTypeDef sDate = {0};

uint8_t Decimal_To_BCD(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

uint8_t BCD_To_Decimal(uint8_t bcd) {
   return ((bcd >> 4) * 10) + (bcd & 0x0F);
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
  ssd1306_Init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  void checkShouldGoToTimer()
  {
    uint8_t rawCounter = __HAL_TIM_GET_COUNTER(&htim2);

    if (rawCounter != prevCounter)
    {
      activeScreen = screenTimer;
      prevCounter = rawCounter;
    }
  }

  void printTimeScreen()
  {
	  checkShouldGoToTimer();

	  HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BCD);
	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, FORMAT_BCD);
	  uint8_t timeHours = BCD_To_Decimal(sTime.Hours);
	  uint8_t timeMinutes = BCD_To_Decimal(sTime.Minutes);
	  uint8_t timeSeconds = BCD_To_Decimal(sTime.Seconds);

	  char bufferHours[4];
	  sprintf(bufferHours, "%02d", timeHours);
	  char bufferMin[4];
	  sprintf(bufferMin, "%02d", timeMinutes);
	  char bufferSec[4];
	  sprintf(bufferSec, "%02d", timeSeconds);

	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteScaledString(bufferHours, Font_16x26, White, 1.6, 1.6);
	  ssd1306_SetCursor(47, 0);
	  ssd1306_WriteScaledString(":", Font_16x26, White, 1.6, 1.6);
	  ssd1306_SetCursor(72, 0);
	  ssd1306_WriteScaledString(bufferMin, Font_16x26, White, 1.6, 1.6);

	  ssd1306_FillRectangle(0, 51, timeSeconds * 2, 55, White);
  }

  void printWeatherScreen()
  {
	  BME280_Measure();
	  HAL_Delay(100);

	  char bufferTemp[10];
      // added -1 to temperature in order not to calibrate it))
	  snprintf(bufferTemp, sizeof(bufferTemp), "%04.1f C", Temperature - 1);

	  char bufferPres[15];
	  snprintf(bufferPres, sizeof(bufferPres), "%05.1f mmpc", Pressure / 133.322);

	  char bufferHum[10];
	  snprintf(bufferHum, sizeof(bufferHum), "%04.1f %%", Humidity);

	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString(bufferTemp, Font_11x18, White);
	  ssd1306_SetCursor(0, 20);
	  ssd1306_WriteString(bufferHum, Font_11x18, White);
	  ssd1306_SetCursor(0, 40);
	  ssd1306_WriteString(bufferPres, Font_11x18, White);
  }

  void printTimerScreen()
  {
	  HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BCD);
	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, FORMAT_BCD);

	  uint32_t rawCounter = __HAL_TIM_GET_COUNTER(&htim2);

    // For now counterMode is not used. 
	  if (counterMode == 0) {
		  counter = rawCounter;
	  } else {
      if (rawCounter == 13) {
        __HAL_TIM_SET_COUNTER(&htim2, 1); // 12 * 5 = 60
        counter = 0;
      } else if (rawCounter == 0) {
        __HAL_TIM_SET_COUNTER(&htim2, 12); // 11 * 5 = 55
        counter = 55;
      } else {
        counter = (rawCounter - 1) * 5;
      }
	  }

	  uint8_t timerSeconds = BCD_To_Decimal(sAlarm.AlarmTime.Seconds);
	  uint8_t timeSeconds = BCD_To_Decimal(sTime.Seconds);
	  uint8_t timerMinutes = BCD_To_Decimal(sAlarm.AlarmTime.Minutes);
	  uint8_t timeMinutes = BCD_To_Decimal(sTime.Minutes);
	  uint8_t timerHours = BCD_To_Decimal(sAlarm.AlarmTime.Hours);
	  uint8_t timeHours = BCD_To_Decimal(sTime.Hours);

    uint32_t totalSecondsAlarm = timerHours * 3600 + timerMinutes * 60 + timerSeconds;
    uint32_t totalSecondsCurrent = timeHours * 3600 + timeMinutes * 60 + timeSeconds;
    uint32_t totalSecondsLeft = totalSecondsAlarm - totalSecondsCurrent;

    uint8_t fullMinutesLeft = (totalSecondsLeft / 60) % 60;
    uint8_t secondsLeft = totalSecondsLeft % 60;

	  char bufferEncoderOrTimer[8];
	  sprintf(bufferEncoderOrTimer, "%02d", alarmIsSet ? fullMinutesLeft : counter);
	  char bufferSecondsLeft[4];
	  sprintf(bufferSecondsLeft, "%02d", secondsLeft);

	  ssd1306_SetCursor(00, 0);
	  ssd1306_WriteScaledString(bufferEncoderOrTimer, Font_16x26, White, 1.6, 1.6);
	  ssd1306_SetCursor(47, 0);
	  ssd1306_WriteScaledString(alarmIsSet ? ":" : "", Font_16x26, White, 1.6, 1.6);
	  ssd1306_SetCursor(72, 0);
	  ssd1306_WriteScaledString(alarmIsSet ? bufferSecondsLeft : "", Font_16x26, White, 1.6, 1.6);
	  ssd1306_SetCursor(3, 45);
	  ssd1306_WriteString("min", Font_11x18, White);
	  ssd1306_SetCursor(75, 45);
	  ssd1306_WriteString(alarmIsSet ? "sec" : "", Font_11x18, White);
  }

  void printMenuScreen()
  {
		uint8_t rawCounter = __HAL_TIM_GET_COUNTER(&htim2);

		if (rawCounter != prevCounter)
		{

			if (activeMenuOption == menuSetTime)
			{
				activeMenuOption = menuSetDate;
			} else {
				activeMenuOption = menuSetTime;
			}

			prevCounter = rawCounter;
		}

    ssd1306_SetCursor(0, 0);
    ssd1306_FillRectangle(0, 0, 128, 20, activeMenuOption == menuSetTime ? White : Black);
    ssd1306_WriteString("Set Time", Font_11x18, activeMenuOption == menuSetTime ? Black : White);

    ssd1306_SetCursor(0, 25);
    ssd1306_FillRectangle(0, 25, 128, 20, activeMenuOption == menuSetDate ? White : Black);
    ssd1306_WriteString("Set Date", Font_11x18, activeMenuOption == menuSetDate ? Black : White);
  }

  void printSetTimeScreen()
  {
	  uint8_t rawCounter = __HAL_TIM_GET_COUNTER(&htim2);

	  if (activeSetTimeOption == setTimeSecondsOption) {
		  setTimeSeconds = rawCounter;
	  } else if (activeSetTimeOption == setTimeMinutesOption) {
		  setTimeMinutes = rawCounter;
	  } else if (activeSetTimeOption == setTimeHoursOption) {
		  setTimeHours = rawCounter;
	  }

	  char bufferHours[4];
	  sprintf(bufferHours, "%02d", setTimeHours);
	  char bufferMin[4];
	  sprintf(bufferMin, "%02d", setTimeMinutes);
	  char bufferSec[4];
	  sprintf(bufferSec, "%02d", setTimeSeconds);

	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString(bufferHours, Font_16x26, White);
	  ssd1306_SetCursor(35, 0);
	  ssd1306_WriteString(":", Font_16x26, White);
	  ssd1306_SetCursor(55, 0);
	  ssd1306_WriteString(bufferMin, Font_16x26, White);
	  ssd1306_SetCursor(95, 0);
	  ssd1306_WriteString(bufferSec, Font_7x10, White);
  }

  void checkAlarm()
  {
    if (isAlarm == 1)
    {
      alarmIterations++;

      if ((alarmIterations % 2) == 0)
      {
        // A1 - buzzer + LED
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
      }
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  ssd1306_Fill(Black);

	  if (activeScreen == screenTime)
	  {
		  printTimeScreen();
	  } else if (activeScreen == screenWeather)
	  {
		  printWeatherScreen();
	  } else if (activeScreen == screenTimer)
	  {
		  printTimerScreen();
	  } else if (activeScreen == screenMenu)
	  {
		  printMenuScreen();
	  } else if (activeScreen == screenSetTime)
	  {
		  printSetTimeScreen();
	  }

	  ssd1306_UpdateScreen();

	  checkAlarm();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef DateToUpdate = {0};
//  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  uint32_t backup_val = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  if (backup_val != 0x32F2)
  {
	  /* USER CODE END Check_RTC_BKUP */

	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0x0;
	  sTime.Minutes = 0x0;
	  sTime.Seconds = 0x0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
		Error_Handler();
	  }
	  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	  DateToUpdate.Month = RTC_MONTH_JANUARY;
	  DateToUpdate.Date = 0x1;
	  DateToUpdate.Year = 0x0;

	  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Enable the Alarm A
	  */
	  sAlarm.AlarmTime.Hours = 0x0;
	  sAlarm.AlarmTime.Minutes = 0x0;
	  sAlarm.AlarmTime.Seconds = 0x0;
	  sAlarm.Alarm = RTC_ALARM_A;
	  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN RTC_Init 2 */

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
} else {
	// do not initialize if we get data from backup
}
  /* USER CODE END RTC_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  HAL_GPIO_WritePin(Alarm_Led_GPIO_Port, Alarm_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Alarm_Led_Pin */
  GPIO_InitStruct.Pin = Alarm_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Alarm_Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Set_RTC_Alarm(uint32_t timerMinutes)
{
  // Get the current time and date
  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
  // Needed to read date as well due to a quirk
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD); 

  // Calculate the new alarm time
  uint32_t minutes = BCD_To_Decimal(sTime.Minutes) + timerMinutes;
  uint32_t new_minutes = minutes % 60;
  uint32_t new_hours = (BCD_To_Decimal(sTime.Hours) + (minutes / 60)) % 24;

  sAlarm.AlarmTime.Hours = Decimal_To_BCD(new_hours);
  sAlarm.AlarmTime.Minutes = Decimal_To_BCD(new_minutes);
  sAlarm.AlarmTime.Seconds = sTime.Seconds;

  // Set the alarm
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }

  // Clear encoder counter
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void Clear_RTC_Alarm()
{
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

	sAlarm.AlarmTime.Hours = sTime.Hours;
	sAlarm.AlarmTime.Minutes = sTime.Minutes;
	sAlarm.AlarmTime.Seconds = sTime.Seconds;

	isAlarm = 0;
	alarmIsSet = 0;
}

volatile uint32_t lastInterruptTime1 = 0;
volatile uint32_t lastInterruptTime3 = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (isAlarm == 1) {
		Clear_RTC_Alarm();
	}	else {
		uint32_t currentTime = HAL_GetTick();

		//	Button 1
		if (GPIO_Pin == GPIO_PIN_3)
		{
			if (currentTime - lastInterruptTime3 > 300)
			{
				if (activeScreen == screenTimer)
				{
					uint8_t rawCounter = __HAL_TIM_GET_COUNTER(&htim2);

					if (rawCounter != 0) {
						if (alarmIsSet == 1) {
							Clear_RTC_Alarm();
						} else {
							Set_RTC_Alarm(rawCounter);
							alarmIsSet = 1;
						}
					} else {
						Clear_RTC_Alarm();
					}
				} else {
					if (activeScreen == screenSetTime) {
						activeScreen = screenMenu;
					} else if (activeScreen == screenMenu) {
						activeScreen = screenTime;
					} else if (activeScreen == screenTime) {
						activeScreen = screenMenu;
					}
				}

				lastInterruptTime3 = currentTime;
			}
		}

		//	Button 2
		if (GPIO_Pin == GPIO_PIN_1)
		{
			if (currentTime - lastInterruptTime1 > 300)
			{
				if (activeScreen == screenTimer)
				{
					activeScreen = screenTime;
				} else if (activeScreen == screenWeather)
				{
					activeScreen = screenTimer;
				} else if (activeScreen == screenTime)
				{
					activeScreen = screenWeather;
				} else if (activeScreen == screenMenu)
				{
					if (activeMenuOption == menuSetTime)
					{
						activeScreen = screenSetTime;
					}
				} else if (activeScreen == screenSetTime) {
					  if (activeSetTimeOption == setTimeSecondsOption) {
						  sTime.Seconds = Decimal_To_BCD(setTimeSeconds);
						  __HAL_TIM_SET_COUNTER(&htim2, 0);
						  activeSetTimeOption = setTimeMinutesOption;
					  } else if (activeSetTimeOption == setTimeMinutesOption) {
						  sTime.Minutes = Decimal_To_BCD(setTimeMinutes);
						  __HAL_TIM_SET_COUNTER(&htim2, 0);
						  activeSetTimeOption = setTimeHoursOption;
					  } else if (activeSetTimeOption == setTimeHoursOption) {
						  sTime.Hours = Decimal_To_BCD(setTimeHours);
						  __HAL_TIM_SET_COUNTER(&htim2, 0);
						  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);
						  activeSetTimeOption = setTimeSecondsOption;
						  activeScreen = screenMenu;
					  }
				}

				lastInterruptTime1 = currentTime;
			}
		}
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	if (alarmIsSet == 1) {
		isAlarm = 1;
		alarmIsSet = 0;
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
