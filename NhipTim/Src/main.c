/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "anh.h"
#include "anh2.h"
#include <time.h>
#include <stdio.h>

#include <math.h>
#include <stdlib.h>
#include "Max30100.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SO_PHAN_TU_LOC_TRUNG_BINH 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

char Rx_data[10] = { 0 };
uint16_t NhipTim, Spo2, LedDo, NhietDo;
char Tx_data[30] = { 0 };
uint64_t lastUart = 0, lastReset =0;
char MangNhipTim[3]="000", MangSpo2[3]="000";
uint16_t test=0, lastBPM=0;

// Bộ lọc trung bình
float values[SO_PHAN_TU_LOC_TRUNG_BINH]={0};
uint8_t  indexx=0;
float sum=0;
uint8_t count=0;

float meanDiffIR=0;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// TODO
uint8_t CountNumber(uint16_t x)
{
	uint8_t count=0;
	while(x%10 !=0)
	{
		count ++;
		x=x/10;
	}
	return count;
}
// TODO DC filter
uint16_t dcRemoval(uint32_t x, uint32_t prev_w, float alpha)
{
	uint32_t result,w ;
	w = x + prev_w*alpha;
	result = w - prev_w;
	return result;
}
// TODO Bộ lọc trung bình
float meanDiff(float M)
{
  float avg = 0;

  sum -= values[indexx];
  values[indexx] = M;
  sum += values[indexx];

  indexx++;
  indexx = indexx % SO_PHAN_TU_LOC_TRUNG_BINH;

  if(count < SO_PHAN_TU_LOC_TRUNG_BINH)
    count++;

  avg = sum / count;
  return avg - M;
}

// CALLBACK
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 8);
//	sprintf(Tx_data, "         A%dB%dC%dD%dE", dcIR, Spo2, rawIR, NhietDo);
//	HAL_UART_Transmit(&huart1, (uint8_t*) Tx_data, strlen(Tx_data), 1000);

	SSD1306_GotoXY(35, 15);
	SSD1306_Puts(Rx_data, &Font_7x10, 1);
	if(HAL_GetTick() - lastReset > 1000)
	{
		HAL_NVIC_SystemReset();
	}
	lastReset= HAL_GetTick();

	LedDo = rawIR/100;
	Spo2 = rawRed/100;
	NhietDo	= Temp;
	sprintf(MangNhipTim, "%u", LedDo);
	sprintf(MangSpo2, "%u", Spo2);
	for(int i=2;i>(CountNumber(Spo2)-1);i--)
	{
		MangSpo2[i]='.';
	}
	for(int i=2;i>(CountNumber(LedDo)-1);i--)
	{
		MangNhipTim[i]='.';
	}
	SSD1306_GotoXY(55, 26);
	SSD1306_Puts(MangNhipTim, &Font_11x18, 1);
	SSD1306_GotoXY(55, 45);
	SSD1306_Puts(MangSpo2, &Font_11x18, 1);
	SSD1306_UpdateScreen();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		MAX30100_InterruptHandler();


		// Bộ lọc DC
		dcIR = dcRemoval(rawIR, dcIR, 0.9);
		dcRed = dcRemoval(rawRed, dcRed, 0.95);
		// Bộ lọc trung bình
		meanDiffIR = meanDiff(dcIR);

		sprintf(Tx_data, "         A%dB%dC%dD%dE", dcIR, Spo2, rawIR, NhietDo);
		HAL_UART_Transmit(&huart1, (uint8_t*) Tx_data, strlen(Tx_data), 1000);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 8); // Nhận Data
// TODO  Oled
	SSD1306_Init();  // initialise Oled
	SSD1306_DrawBitmap(100, 0, pin3, 128, 64, 1);
	SSD1306_GotoXY(35, 15);
	SSD1306_Puts("00:00:00", &Font_7x10, 1);
	SSD1306_DrawBitmap(3, 23, tim, 128, 64, 1);
	SSD1306_GotoXY(30, 26);
	SSD1306_Puts(":", &Font_11x18, 1);
	SSD1306_GotoXY(55, 26);
	SSD1306_Puts(MangNhipTim, &Font_11x18, 1);
	SSD1306_GotoXY(100, 33);
	SSD1306_Puts("BPM", &Font_7x10, 1);
	SSD1306_GotoXY(30, 45);
	SSD1306_Puts(":", &Font_11x18, 1);
	SSD1306_DrawBitmap(4, 45, oxi, 128, 64, 1);
	SSD1306_GotoXY(55, 45);
	SSD1306_Puts(MangSpo2, &Font_11x18, 1);
	SSD1306_GotoXY(100, 45);
	SSD1306_Puts("%", &Font_11x18, 1);
	//	      SSD1306_Clear();
	SSD1306_DrawBitmap(0, 0, wifiThuNho, 128, 40, 1);
	SSD1306_UpdateScreen();

	srand(time(NULL));
	lastUart = HAL_GetTick();
	lastReset= HAL_GetTick();

	// Max30100
	MAX30100_Init(&hi2c1);
	MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
	MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT, MAX30100_LEDCURRENT_DEFAULT);
	MAX30100_SetMode(MAX30100_SPO2_MODE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (HAL_GetTick() - lastUart > 100) {
			NhipTim = rand() % 90;
			Spo2 = rand() % 80;
			lastUart = HAL_GetTick();
		}


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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
