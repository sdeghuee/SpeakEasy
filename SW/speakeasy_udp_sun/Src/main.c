
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "udp_sun.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// timers
uint32_t msCount = 0;
uint8_t started = 0;

// button debounce
uint8_t buttonPress[5] = {0, 0, 0, 0, 0};
uint8_t buttonPrev[5] = {0, 0, 0, 0, 0};

// ethernet
extern struct netif gnetif;
uint8_t bufferReceived[10];
uint16_t playbackControlRx = NOP;
uint16_t playbackControl[2];

// audio
uint8_t playing = 1;
uint8_t adcDone = 0;
uint16_t audioData = 0;
uint16_t txBuffer[10 * (BUFF_SIZE + 1)];
uint32_t txIndex = 0;
uint8_t playbackStarted = 0;
uint16_t playbackIndex = 0;

// begin ryan
double ampFactor = 1;
uint16_t adcOffset = 2187;

//I2C INIT VARIABLES
uint8_t i2cAddress = 124;
uint8_t i2cMinResistance[2] = {0x00,0x00};
uint8_t i2cDefaultResistance[2] = {0x00, 0x0c};
uint8_t i2cErr = 0;
uint16_t minResHoldReq = 75;
uint16_t maxResHoldReq = 75;
uint16_t msBetweenToggle = 0;
uint8_t toggleNum = 0;

//uint8_t convCplt = 0;
//uint16_t convCount = 0;
//uint16_t convSum = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void debounceButton();
void checkButtonPress();
void checkADC();
void checkPlaybackControl();

// begin ryan
uint16_t amplify(uint16_t, double);
void I2C_SetMinResistance();
void I2C_SetDefaultResistance();
void Butt_PP();
void Butt_FF();
void Butt_RW();
void Butt_VolUp();
void Butt_VolDown();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(AMP_Gain0_GPIO_Port, AMP_Gain0_Pin);
		if (playbackStarted) {
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, txBuffer[playbackIndex++]);
			if (playbackIndex == 10 * BUFF_SIZE) {
				playbackIndex = 0;
			}
		}
		HAL_GPIO_WritePin(AMP_Gain1_GPIO_Port, AMP_Gain1_Pin, 1);
		HAL_ADC_Start_IT(&hadc1);
//		if (started) {
//			audioData = convSum / convCount;
//			adcDone = 1;
//		}
//		convSum = 0;
//		convCount = 0;
//		convCplt = 1;
	}
	else if (htim->Instance == TIM3) {
		debounceButton();
		if (++msCount == 1000) {
			msCount = 0;
			if (!started) {
				started = 1;
				HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, 0);
				HAL_GPIO_WritePin(LED_Amber_GPIO_Port, LED_Amber_Pin, 0);
			}
		}
		if (++msBetweenToggle > 1000) {
			msBetweenToggle = 1000;
		}
		if(toggleNum > 0){
			switch(toggleNum % 2){
				case 1:
					if(msBetweenToggle >= minResHoldReq){
						I2C_SetDefaultResistance();
						msBetweenToggle = 0;
						toggleNum--;
					}
					break;
				case 0:
					if(msBetweenToggle >= maxResHoldReq){
						I2C_SetMinResistance();
						msBetweenToggle = 0;
						toggleNum--;
					}
					break;
			}
		}
//		if(i2cErr&&msCount == 0){
//			HAL_GPIO_TogglePin(LED_Amber_GPIO_Port,LED_Amber_Pin);
//			HAL_GPIO_TogglePin(LED_Red_GPIO_Port,LED_Red_Pin);
//		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc) {
	HAL_GPIO_WritePin(AMP_Gain1_GPIO_Port, AMP_Gain1_Pin, 0);
	audioData = HAL_ADC_GetValue(hadc);
	audioData = amplify(audioData, ampFactor);
	adcDone = 1;
//	convSum += HAL_ADC_GetValue(hadc);
//	convCount++;
//	if (convCount < 1) {
//		convCplt = 1;
//	}
}

void debounceButton() {
	uint8_t buttonIndex = msCount % 5;
	uint8_t buttonCurrent;
	switch (buttonIndex) {
		case 0:
			buttonCurrent = HAL_GPIO_ReadPin(PB_VolD_GPIO_Port, PB_VolD_Pin);
			break;
		case 1:
			buttonCurrent = HAL_GPIO_ReadPin(PB_Prev_GPIO_Port, PB_Prev_Pin);
			break;
		case 2:
			buttonCurrent = HAL_GPIO_ReadPin(PB_PlayPause_GPIO_Port, PB_PlayPause_Pin);
			break;
		case 3:
			buttonCurrent = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
//			buttonCurrent = HAL_GPIO_ReadPin(PB_Next_GPIO_Port, PB_Next_Pin);
			break;
		case 4:
			buttonCurrent = HAL_GPIO_ReadPin(PB_VolU_GPIO_Port, PB_VolU_Pin);
			break;
		default:
			buttonCurrent = 0;
			break;
	}
	buttonPress[buttonIndex] = !buttonPrev[buttonIndex] && buttonCurrent;
	buttonPrev[buttonIndex] = buttonCurrent;
}

void checkButtonPress() {
	for (uint8_t i = 0; i < 5; i++) {
	  if (buttonPress[i]) {
		  switch (i) {
			  case 0:
				  Butt_VolDown();
				  break;
			  case 1:
				  if (!toggleNum) {
					  Butt_RW();
				  }
				  break;
			  case 2:	//Play Pause
				  if (!toggleNum) {
					  Butt_PP();
				  }
				  playing ^= 1;
				  playbackControl[0] = Pause - playing;
				  udp_scratch_send(&playbackControl[0], 2);
				  break;
			  case 3:
				  buttonPress[3] = 0;
				  if (!toggleNum) {
					  Butt_FF();
				  }
				  break;
			  case 4:
				  Butt_VolUp();
				  break;
			  default:
				  break;
		  }
	  }
	  buttonPress[i] = 0;
	}
}

void checkADC() {
	if (adcDone && started) {
		adcDone = 0;
		txBuffer[txIndex++] = audioData;
		if (txIndex % BUFF_SIZE == 0) {
			bufferReceived[(txIndex - 1) / BUFF_SIZE] = 0;
			udp_scratch_send_audio(&txBuffer[txIndex - BUFF_SIZE], BUFF_SIZE, (txIndex - 1) / BUFF_SIZE);
			if (!playbackStarted && txIndex == 6 * BUFF_SIZE) {
				playbackStarted = 1;
			}
			if (txIndex == 10 * BUFF_SIZE) {
				txIndex = 0;
			}
		}
	}
}

void checkPlaybackControl() {
	if (playbackControlRx != NOP) {
		switch (playbackControlRx) {
			case Vol_D:
				Butt_VolDown();
				break;
			case Prev:
				Butt_RW();
				break;
			case Play:
				Butt_PP();
				playing = 1;
				break;
			case Pause:
				Butt_PP();
				playing = 0;
				break;
			case Next:
				Butt_FF();
				break;
			case Vol_U:
				Butt_VolUp();
				break;
		}
		playbackControlRx = NOP;
//		HAL_GPIO_WritePin(AMP_Mute_GPIO_Port, AMP_Mute_Pin, playing);
	}
}

uint16_t amplify(uint16_t in, double factor){
	double out=adcOffset;
	if(in > adcOffset){
		out+=(in-adcOffset)*factor;
	}else{
		out-=(adcOffset-in)*factor;
	}
	return (uint16_t) out;
}
void I2C_SetMinResistance(){
	HAL_I2C_Master_Transmit(&hi2c3,i2cAddress,i2cMinResistance,2,10);
}
void I2C_SetDefaultResistance(){
	HAL_I2C_Master_Transmit(&hi2c3,i2cAddress,i2cDefaultResistance,2,10);
}
void Butt_PP(){
	toggleNum = 2;
}
void Butt_FF(){
	toggleNum = 4;
}
void Butt_RW(){
	toggleNum = 6;
}
void Butt_VolUp(){
	if(ampFactor < 1.5)
		ampFactor+=.1;
}
void Butt_VolDown(){
	if(ampFactor > .5)
		ampFactor-=.1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_LWIP_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
	err_t errout = ethernetif_init(&gnetif);
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, errout != ERR_OK);
	HAL_GPIO_WritePin(LED_Amber_GPIO_Port, LED_Amber_Pin, netif_is_up(&gnetif));
	udp_scratch_connect();
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	I2C_SetDefaultResistance();
	playbackControl[1] = NOP;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		ethernetif_set_link(&gnetif);
		ethernetif_input(&gnetif);
		checkButtonPress();
		checkADC();
		checkPlaybackControl();
//		if (convCplt) {
//			convCplt = 0;
//			HAL_GPIO_WritePin(AMP_Gain1_GPIO_Port, AMP_Gain1_Pin, 1);
//			HAL_ADC_Start_IT(&hadc1);
//		}
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_32K;
  hi2s2.Init.CPOL = I2S_CPOL_HIGH;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 950;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 953;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 88;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1309;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, AMP_Gain0_Pin|AMP_Gain1_Pin|AMP_Mute_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ADC_Switch_Pin|LED_Red_Pin|LED_Amber_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PHY_Reset_GPIO_Port, PHY_Reset_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : AMP_Gain0_Pin AMP_Gain1_Pin AMP_Mute_Pin */
  GPIO_InitStruct.Pin = AMP_Gain0_Pin|AMP_Gain1_Pin|AMP_Mute_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MII_INT_Pin */
  GPIO_InitStruct.Pin = MII_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MII_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB_PlayPause_Pin PD15 PHY_RX_ERR_Pin */
  GPIO_InitStruct.Pin = PB_PlayPause_Pin|GPIO_PIN_15|PHY_RX_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_Switch_Pin LED_Red_Pin LED_Amber_Pin */
  GPIO_InitStruct.Pin = ADC_Switch_Pin|LED_Red_Pin|LED_Amber_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB_VolD_Pin PC7 PC8 */
  GPIO_InitStruct.Pin = PB_VolD_Pin|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PB_VolU_Pin PB_Next_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|PB_VolU_Pin|PB_Next_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PHY_Reset_Pin */
  GPIO_InitStruct.Pin = PHY_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PHY_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_Prev_Pin */
  GPIO_InitStruct.Pin = PB_Prev_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_Prev_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
