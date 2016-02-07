/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "battery-socket.h"
#include "arm_math.h"
#include "arm_common_tables.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

DCMI_HandleTypeDef hdcmi;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

NOR_HandleTypeDef hnor1;
SRAM_HandleTypeDef hsram2;
SRAM_HandleTypeDef hsram3;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
VoltageStruct voltageStruct;

static DAC_ChannelConfTypeDef sConfig;

const uint8_t aEscalator8bit[6] = { 0x0, 0x33, 0x66, 0x99, 0xCC, 0xFF };

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue[VOLTAGE_BUFFER_LENGTH] = { 0 };
__IO uint16_t uhDACxConvertedValue = 0;
uint32_t sampleCounter = 0;
uint32_t sampleMultiplier = 0;
uint32_t bufferFirst[ETHERNET_BUFFER_LENGTH] = { 0 };
uint32_t bufferLast[ETHERNET_BUFFER_LENGTH] = { 0 };

#define sinBufferSize 1024
__IO uint16_t sinBuffer[sinBufferSize] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_DCMI_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void DAC_Ch1_SinConfig(void);

static void ToggleLed4(void const * argument);
static void BatteryVoltageMonitor(void const * argument);
static void BatteryVoltageController(void const * argument);
static void Error_Handler(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle);
void HAL_ADC_ConvCpltCallback2(ADC_HandleTypeDef* AdcHandle);
void SET_DAC();

static void DAC_Ch1_TriangleConfig(void);
static void DAC_Ch1_EscalatorConfig(void);
void InitializeVoltageStruct();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	voltageStruct.packetHeader = 0x0000AA55;
	voltageStruct.bufferFirstHalf = bufferFirst;
	voltageStruct.bufferLastHalf = bufferLast;
	voltageStruct.bufferLength = ETHERNET_BUFFER_LENGTH;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_DCMI_Init();
  MX_FSMC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_USB_OTG_HS_USB_Init();

  /* USER CODE BEGIN 2 */

	/*##-2- Enable TIM peripheral counter ######################################*/
	HAL_TIM_Base_Start(&htim6);

	/*##-3- Start the conversion process and enable interrupt ##################*/
	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &uhADCxConvertedValue,
	VOLTAGE_BUFFER_LENGTH) != HAL_OK) {
		/* Start Conversation Error */
		Error_Handler();
	}

	DAC_Ch1_SinConfig();

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

//  osThreadDef(LED4, ToggleLed4, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(LED4), NULL);
//  osThreadDef(BATTERYADC1, BatteryVoltageMonitor, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(BATTERYADC1), NULL);
//  osThreadDef(BATTERYDAC1, BatteryVoltageController, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//  osThreadCreate(osThread(BATTERYDAC1), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC3 init function */
void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc3);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* DCMI init function */
void MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  HAL_DCMI_Init(&hdcmi);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_ALARMA;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BCD);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm(&hrtc, &sAlarm, FORMAT_BCD);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  HAL_SD_Init(&hsd, &SDCardInfo);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFF;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/* USB_OTG_FS init function */
void MX_USB_OTG_FS_USB_Init(void)
{

}

/* USB_OTG_HS init function */
void MX_USB_OTG_HS_USB_Init(void)
{

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins
     PE2   ------> SYS_TRACECLK
     PB5   ------> USB_OTG_HS_ULPI_D7
     PE5   ------> SYS_TRACED2
     PE6   ------> SYS_TRACED3
     PA12   ------> USB_OTG_FS_DP
     PI3   ------> I2S2_SD
     PA11   ------> USB_OTG_FS_DM
     PA10   ------> USB_OTG_FS_ID
     PI11   ------> USB_OTG_HS_ULPI_DIR
     PI0   ------> I2S2_WS
     PA9   ------> USB_OTG_FS_VBUS
     PA8   ------> RCC_MCO_1
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PG7   ------> USART6_CK
     PC0   ------> USB_OTG_HS_ULPI_STP
     PA5   ------> USB_OTG_HS_ULPI_CK
     PB12   ------> USB_OTG_HS_ULPI_D5
     PB13   ------> USB_OTG_HS_ULPI_D6
     PA3   ------> USB_OTG_HS_ULPI_D0
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PB10   ------> USB_OTG_HS_ULPI_D3
     PB11   ------> USB_OTG_HS_ULPI_D4
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();

  /*Configure GPIO pins : TRACE_CLK_Pin TRACE_D2_Pin TRACE_D3_Pin */
  GPIO_InitStruct.Pin = TRACE_CLK_Pin|TRACE_D2_Pin|TRACE_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D5_Pin ULPI_D6_Pin ULPI_D2_Pin 
                           ULPI_D1_Pin ULPI_D3_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D5_Pin|ULPI_D6_Pin|ULPI_D2_Pin 
                          |ULPI_D1_Pin|ULPI_D3_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SmartCard_CMDVCC_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = SmartCard_CMDVCC_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_DP_Pin USB_FS_DM_Pin USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_DP_Pin|USB_FS_DM_Pin|USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S_SD_Pin I2S_WS_Pin */
  GPIO_InitStruct.Pin = I2S_SD_Pin|I2S_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_Expander_INT_Pin */
  GPIO_InitStruct.Pin = IO_Expander_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO_Expander_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SmartCard_3_5V_Pin OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = SmartCard_3_5V_Pin|OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MicroSDCard_Detect_Pin */
  GPIO_InitStruct.Pin = MicroSDCard_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MicroSDCard_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

//  /*Configure GPIO pin : MCO_Pin */
//  GPIO_InitStruct.Pin = MCO_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
//  HAL_GPIO_Init(MCO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SmartCard_CLK_Pin */
  GPIO_InitStruct.Pin = SmartCard_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(SmartCard_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SmartCard_RST_Pin */
  GPIO_InitStruct.Pin = SmartCard_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(SmartCard_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SmartCard_OFF_Pin */
  GPIO_InitStruct.Pin = SmartCard_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SmartCard_OFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_STP_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_STP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MII_INT_Pin */
  GPIO_InitStruct.Pin = MII_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MII_INT_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the NOR1 memory initialization sequence
  */
  hnor1.Instance = FSMC_NORSRAM_DEVICE;
  hnor1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor1.Init */
  hnor1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hnor1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FSMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hnor1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hnor1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hnor1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hnor1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hnor1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  HAL_NOR_Init(&hnor1, &Timing, NULL);

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FSMC_NORSRAM_DEVICE;
  hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FSMC_NORSRAM_BANK2;
  hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  HAL_SRAM_Init(&hsram2, &Timing, NULL);

  /** Perform the SRAM3 memory initialization sequence
  */
  hsram3.Instance = FSMC_NORSRAM_DEVICE;
  hsram3.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram3.Init */
  hsram3.Init.NSBank = FSMC_NORSRAM_BANK3;
  hsram3.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram3.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram3.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram3.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram3.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram3.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram3.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram3.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram3.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram3.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram3.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram3.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  HAL_SRAM_Init(&hsram3, &Timing, NULL);

}

/* USER CODE BEGIN 4 */

//uint32_t dacCounter = 0;
//float32_t dacPeriod= 0x7FF / 50000000.;
//float32_t dacResolution = 2^12;
/////**
////  * @brief  Conversion complete callback in non blocking mode for Channel1
////  * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
////  *         the configuration information for the specified DAC.
////  * @retval None
////  */
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
//{
//  float32_t theta = dacCounter * 2. * 3.14 * dacPeriod;
//  uint32_t sinTheta = dacResolution * (1.001 + arm_sin_f32(theta)) / 2.;
//
////  uhDACxConvertedValue = (uint32_t)sinTheta;
//  dacCounter += 1;
////  HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 2048);
//}
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim: TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	broadcastVoltageAll();
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

static void BatteryVoltageMonitor(void const * argument) {

	/*##-3- Start the conversion process and enable interrupt ##################*/
	if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &uhADCxConvertedValue,
	VOLTAGE_BUFFER_LENGTH) != HAL_OK) {
		/* Start Conversation Error */
		Error_Handler();
	}

	/* Infinite loop */
	while (1) {
		osDelay(1);
	}

}

static void BatteryVoltageController(void const * argument) {
//      DAC_Ch1_TriangleConfig();
//      DAC_Ch1_EscalatorConfig();

	/* Infinite loop */
	while (1) {
		osDelay(1);
	}

}

/**
 * @brief  Conversion complete callback in non blocking mode for Channel1
 * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
 *         the configuration information for the specified DAC.
 * @retval None
 */
//void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac)
//{
//	q15_t phase = 10 * HAL_GetTick() / 1000;
//	waveformValue = arm_sin_q15(phase);
//	HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, waveformValue);
//	HAL_DAC_Start(hdac, DAC_CHANNEL_1);
//	waveformValue = HAL_GetTick() % 4095;
//	if(HAL_DAC_Start_DMA(hdac, DAC_CHANNEL_1, (uint32_t*)&waveformValue, sizeof(uint16_t), DAC_ALIGN_12B_R) != HAL_OK)
//	{
//	/* Start Error */
//	Error_Handler();
//	}
//}
/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void) {
	/* Turn LED5 on */
//  BSP_LED_On(LED5);
	while (1) {
	}
}

static void DAC_Ch1_EscalatorConfig(void) {
	/*##-1- Initialize the DAC peripheral ######################################*/
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-1- DAC channel1 Configuration #########################################*/
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		/* Channel configuration Error */
		Error_Handler();
	}

	/*##-2- Enable DAC Channel1 and associated DMA #############################*/
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) aEscalator8bit, 6,
	DAC_ALIGN_8B_R) != HAL_OK) {
		/* Start DMA Error */
		Error_Handler();
	}
}

/**
 * @brief  DAC Channel1 Triangle Configuration
 * @param  None
 * @retval None
 */
static void DAC_Ch1_TriangleConfig(void) {
	/*##-1- Initialize the DAC peripheral ######################################*/
	if (HAL_DAC_Init(&hdac) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- DAC channel2 Configuration #########################################*/
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		/* Channel configuration Error */
		Error_Handler();
	}

	/*##-3- DAC channel2 Triangle Wave generation configuration ################*/
	if (HAL_DACEx_TriangleWaveGenerate(&hdac, DAC_CHANNEL_1,
	DAC_TRIANGLEAMPLITUDE_1023) != HAL_OK) {
		/* Triangle wave generation Error */
		Error_Handler();
	}

	/*##-4- Enable DAC Channel1 ################################################*/
	if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}

	/*##-5- Set DAC channel1 DHR12RD register ################################################*/
	if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x100)
			!= HAL_OK) {
		/* Setting value Error */
		Error_Handler();
	}
}

//uint32_t * getVoltagePacket()
//{
//	int i;
//	for(i = 0; i < VOLTAGE_BUFFER_LENGTH; i++)
//	{
//		voltagePacket[i] = voltageStruct.buffer[i];
//		voltageStruct.buffer[i] = 0;
//	}
//	voltageStruct.packetCount = 0;
//	return voltagePacket;
//}

int i, firstHalfBufferIsActive = 1;
uint32_t tick = 0;
/**
 * @brief  Conversion complete callback in non blocking mode
 * @param  AdcHandle : AdcHandle handle
 * @note   This example shows a simple way to report end of conversion, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	int j, startIndex, bufferAverage = 0;

	startIndex = VOLTAGE_BUFFER_LENGTH / 2;
	for (j = startIndex; j < VOLTAGE_BUFFER_LENGTH; j++) {
		bufferAverage += uhADCxConvertedValue[j];
	}

	if (firstHalfBufferIsActive) {
		voltageStruct.bufferFirstHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferFirstHalf[i + 1] = tick
				* (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferFirstHalf[i + 2] = bufferAverage
				/ (VOLTAGE_BUFFER_LENGTH / 2);
	} else {
		voltageStruct.bufferLastHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferLastHalf[i + 1] = tick
				* (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferLastHalf[i + 2] = bufferAverage
				/ (VOLTAGE_BUFFER_LENGTH / 2);
	}
	tick += 1;

	if (i < ETHERNET_BUFFER_LENGTH - 3) {
		i += 3;
	} else {
		i = 0;
		firstHalfBufferIsActive = !firstHalfBufferIsActive;
	}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	int j, endIndex, bufferAverage = 0;

	endIndex = VOLTAGE_BUFFER_LENGTH / 2;
	for (j = 0; j < endIndex; j++) {
		bufferAverage += uhADCxConvertedValue[j];
	}

	if (firstHalfBufferIsActive) {
		voltageStruct.bufferFirstHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferFirstHalf[i + 1] = tick
				* (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferFirstHalf[i + 2] = bufferAverage
				/ (VOLTAGE_BUFFER_LENGTH / 2);
	} else {
		voltageStruct.bufferLastHalf[i] = voltageStruct.packetHeader;
		voltageStruct.bufferLastHalf[i + 1] = tick
				* (VOLTAGE_BUFFER_LENGTH / 2);
		voltageStruct.bufferLastHalf[i + 2] = bufferAverage
				/ (VOLTAGE_BUFFER_LENGTH / 2);
	}
	tick += 1;

	if (i < ETHERNET_BUFFER_LENGTH - 3) {
		i += 3;
	} else {
		i = 0;
		firstHalfBufferIsActive = !firstHalfBufferIsActive;
	}
}

//void HAL_ADC_ConvCpltCallback2(ADC_HandleTypeDef* AdcHandle)
//{
//	averagingBuffer += uhADCxConvertedValue;
//    if(sampleCounter >= VOLTAGE_SUBSAMPLE)
//    {
//    	uint32_t newHeaderLocation = voltageStruct.packetCount * voltageStruct.packetLength;
//    	if(newHeaderLocation + voltageStruct.packetLength <= voltageStruct.bufferLength)
//    	{
//			voltageStruct.buffer[newHeaderLocation] = voltageStruct.packetHeader;
//			voltageStruct.buffer[newHeaderLocation + 1] = sampleMultiplier - VOLTAGE_SUBSAMPLE / 2;
//			voltageStruct.buffer[newHeaderLocation + 2] = averagingBuffer / VOLTAGE_SUBSAMPLE;
//			voltageStruct.packetCount += 1;
//    	}
//
//    	sampleMultiplier += 1;
//
//    	sampleCounter = 0;
//    	averagingBuffer = 0;
//
//    } else
//	{
//		sampleCounter += 1;
//	}
//
//    if(tick >= PERIOD)
//    {
//    	tick = 0;
//    }else
//    {
//    	tick += 1;
//    }
//
//    float32_t phase = ((float32_t)tick) * 2. * 3.14 /((float32_t)PERIOD);
//    float32_t sin = arm_sin_f32(phase * 10.0);
//    float32_t cos = arm_cos_f32(phase);
//
//    uint16_t waveformValue = 4095. * (2. +  .3 * sin + cos) / 4.;
//
//    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, waveformValue);
//    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
//
//}
//

/**
 * @brief  Error DAC callback for Channel1.
 * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
 *         the configuration information for the specified DAC.
 * @retval None
 */
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac) {
	Error_Handler();
}

/**
 * @brief  DMA underrun DAC callback for channel1.
 * @param  hdac: pointer to a DAC_HandleTypeDef structure that contains
 *         the configuration information for the specified DAC.
 * @retval None
 */
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac) {
	Error_Handler();
}

#define PERIOD 2520
static void DAC_Ch1_SinConfig(void) {
	float32_t phase, sin, cos;

	uint16_t waveformValue;

	int i;
	for (i = 0; i < sinBufferSize; i++) {
		phase = ((float32_t) i) * 2. * 3.14 / ((float32_t) sinBufferSize);
		sin = arm_sin_f32(phase * 10.0);
		cos = arm_cos_f32(phase);

		waveformValue = 4095. * (2. + .3 * sin + cos) / 4.;
		sinBuffer[i] = waveformValue;
	}

	/*##-2- Enable DAC selected channel and associated DMA #############################*/
	if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *) sinBuffer,
			(size_t) sinBufferSize, DAC_ALIGN_12B_R) != HAL_OK) {
		/* Start DMA Error */
		Error_Handler();
	}
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN 5 */

	voltage_server_socket();

  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
