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
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO
#define SERVOMAX 875
#define SERVOMIN 975

#define MAX_TOP_RIGHT_TREN 870
#define MIN_TOP_RIGHT_TREN 930

#define MAX_BOT_RIGHT_TREN 895
#define MIN_BOT_RIGHT_TREN 960

#define MAX_TOP_RIGHT_DUOI 875
#define MIN_TOP_RIGHT_DUOI 912

#define MAX_BOT_RIGHT_DUOI 875
#define MIN_BOT_RIGHT_DUOI 912

#define MAX_TOP_LEFT_DUOI 960
#define MIN_TOP_LEFT_DUOI 912

#define MAX_BOT_LEFT_DUOI 960
#define MIN_BOT_LEFT_DUOI 912

#define MAX_TOP_LEFT_TREN 960
#define MIN_TOP_LEFT_TREN 890

#define MAX_BOT_LEFT_TREN 940
#define MIN_BOT_LEFT_TREN 875

// BAN �?ẦU CÂN BẰNG
#define BALANCE_TOP_RIGHT_TREN 900
#define BALANCE_BOT_RIGHT_TREN 930
#define BALANCE_TOP_LEFT_TREN 930
#define BALANCE_BOT_LEFT_TREN 900

#define BALANCE_TOP_RIGHT_DUOI 910
#define BALANCE_TOP_LEFT_DUOI 940
#define BALANCE_BOT_LEFT_DUOI 910
#define BALANCE_BOT_RIGHT_DUOI 910

#define CHANGE_TOP_RIGHT_TREN(TopRightTren)   	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,TopRightTren)
#define CHANGE_TOP_RIGHT_DUOI(TopRightDuoi)   	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,TopRightDuoi)
#define CHANGE_BOT_RIGHT_TREN(BotRightTren)   	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,BotRightTren)
#define CHANGE_BOT_RIGHT_DUOI(BotRightDuoi)   	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,BotRightDuoi)

#define CHANGE_TOP_LEFT_DUOI(TopLeftDuoi)   	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,TopLeftDuoi)
#define CHANGE_TOP_LEFT_TREN(TopLeftTren)   	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,TopLeftTren)
#define CHANGE_BOT_LEFT_TREN(BotLeftTren)   	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,BotLeftTren)
#define CHANGE_BOT_LEFT_DUOI(BotLeftDuoi)   	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,BotLeftDuoi)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// TODO
uint16_t TopRightTren = BALANCE_TOP_RIGHT_TREN, TopRightDuoi =
BALANCE_TOP_RIGHT_DUOI, BotRightTren = BALANCE_BOT_RIGHT_TREN, BotRightDuoi =
		BALANCE_BOT_RIGHT_DUOI, TopLeftTren =
BALANCE_TOP_LEFT_TREN, TopLeftDuoi = BALANCE_TOP_LEFT_DUOI, BotLeftTren =
		BALANCE_BOT_LEFT_TREN, BotLeftDuoi = BALANCE_BOT_LEFT_DUOI;
uint8_t StateMachine=1,State=0, DieuKhien =0;
uint32_t LastTime=0;

char Rx_data[12] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DogForward() {

	for( int i =1; i<=20 ;i++)
	{
		HAL_Delay(25);
		TopRightTren -= 2;
		TopRightDuoi = 895;
		BotRightTren += 2;

		if(i%2==0) BotRightDuoi +=2;
		else BotRightDuoi +=1;

		if(i%2 ==0) TopLeftTren -= 2;
		else TopLeftTren -= 1;

		TopLeftDuoi = 930;

		BotLeftTren += 2;

		if(i%2 ==0) BotLeftDuoi -= 2;
		else BotLeftDuoi -= 1;

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);

	}

	HAL_Delay(100);

	for( int i =1; i<=20 ;i++)
	{
		HAL_Delay(25);
		TopRightTren += 2;
		TopRightDuoi = 926;
		BotRightTren -= 2;

		if(i%2==0) BotRightDuoi -=2;
		else BotRightDuoi -=1;

		if(i%2 ==0) TopLeftTren += 2;
		else TopLeftTren += 1;

		TopLeftDuoi = 967;
		BotLeftTren -= 2;

		if(i%2 ==0) BotLeftDuoi += 2;
		else BotLeftDuoi += 1;

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);

	}

}

// TODO
void DogBalance()
{
	TopRightTren = BALANCE_TOP_RIGHT_TREN;
	TopRightDuoi = BALANCE_TOP_RIGHT_DUOI;
	BotRightTren = BALANCE_BOT_RIGHT_TREN;
	BotRightDuoi = BALANCE_BOT_RIGHT_DUOI;
	TopLeftTren = BALANCE_TOP_LEFT_TREN;
	TopLeftDuoi = BALANCE_TOP_LEFT_DUOI;
	BotLeftTren = BALANCE_BOT_LEFT_TREN;
	BotLeftDuoi = BALANCE_BOT_LEFT_DUOI;


	CHANGE_TOP_RIGHT_TREN(TopRightTren);
	CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
	CHANGE_BOT_RIGHT_TREN(BotRightTren);
	CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);

	CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	CHANGE_TOP_LEFT_TREN(TopLeftTren);
	CHANGE_BOT_LEFT_TREN(BotLeftTren);
	CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
}
void DogTurnLeft()
{
	for(int i =1;i<=20;i++)
	{
		HAL_Delay(25);
		BotRightDuoi = 910;
		TopLeftDuoi = 940;
		BotRightTren +=1;  // 940
		TopLeftTren +=1; // 940

		TopRightDuoi = 890;
		BotLeftDuoi = 930;
		TopRightTren -=1; // 890
		BotLeftTren -=1; // 890

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);

		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
	}

	for(int i =1;i<=20;i++)
	{
		HAL_Delay(25);
		BotRightDuoi = 890;
		TopLeftDuoi = 960;
		BotRightTren -=1;  // 920
		TopLeftTren -=1; // 920

		TopRightDuoi = 910;
		BotLeftDuoi = 910;
		TopRightTren +=1; // 890
		BotLeftTren +=1; // 890

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);

		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
	}



}

void DogTurnRight()
{
	for(int i =1;i<=20;i++)
	{
		HAL_Delay(25);
		BotRightDuoi = 910;
		TopLeftDuoi = 940;
		BotRightTren -=1;  // 940
		TopLeftTren -=1; // 940

		TopRightDuoi = 890;
		BotLeftDuoi = 930;
		TopRightTren +=1; // 890
		BotLeftTren +=1; // 890

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);

		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
	}
	for(int i =1;i<=20;i++)
	{
		HAL_Delay(25);
		BotRightDuoi = 890;
		TopLeftDuoi = 960;
		BotRightTren +=1;  // 920
		TopLeftTren +=1; // 920

		TopRightDuoi = 910;
		BotLeftDuoi = 910;
		TopRightTren -=1; // 890
		BotLeftTren -=1; // 890

		CHANGE_TOP_RIGHT_TREN(TopRightTren);
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		CHANGE_BOT_RIGHT_TREN(BotRightTren);
		CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);

		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
		CHANGE_TOP_LEFT_TREN(TopLeftTren);
		CHANGE_BOT_LEFT_TREN(BotLeftTren);
		CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
	}
}

void DogHello()
{
	for (int i=1 ;i<=20;i++)
	{
		HAL_Delay(15);
		TopRightTren -=2;
		CHANGE_TOP_RIGHT_TREN(TopRightTren);
	}
	for (int i=1 ;i<=20;i++)
	{
		HAL_Delay(15);
		TopRightTren +=2;
		CHANGE_TOP_RIGHT_TREN(TopRightTren);
	}

}
void DogHitDat()
{
	for( int i=1;i<=20;i++)
	{
		HAL_Delay(15);
		if(i%2==0) TopRightDuoi -=2;
		else TopRightDuoi -=1;
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		if(i%2==0) TopLeftDuoi +=2;
		else TopLeftDuoi +=1;
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	}
	for( int i=1;i<=20;i++)
	{
		HAL_Delay(15);
		if(i%2==0) TopRightDuoi +=2;
		else TopRightDuoi +=1;
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		if(i%2==0) TopLeftDuoi -=2;
		else TopLeftDuoi -=1;
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	}

}
void DogDapChan()
{
	for(int i=1;i<=15;i++)
	{
		HAL_Delay(15);
		TopRightDuoi +=1; // 890
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		TopLeftDuoi +=1; // 975
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	}
	for(int i=1;i<=15;i++)
	{
		HAL_Delay(15);
		TopRightDuoi -=1; // 890
		CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
		TopLeftDuoi -=1; // 975
		CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	}

}
void DogKeu()
{
	HAL_GPIO_TogglePin(COI_GPIO_Port, COI_Pin);
	HAL_Delay(100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 12);
	if(Rx_data[1] == '1')  // Điều KHiển
	{
		DieuKhien =1;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	}
	else{     // Tự Động
		DieuKhien =0;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	}

	if(DieuKhien ==1)
	{
		if(Rx_data[6] == '1')  // thẳng
		{
			StateMachine = 1;
		}
		else if(Rx_data[6] == '2')  // im
		{
			StateMachine =0;
		}
		else if(Rx_data[6] == '3')  // trái
		{
			StateMachine =2;
		}
		else if(Rx_data[6] == '4') // phải
		{
			StateMachine =3;
		}
		else if(Rx_data[8] == '1') // vãy tay
		{
			StateMachine =4;
		}
		else if(Rx_data[8] == '2') // hit dat
		{
			StateMachine =5;
		}
		else if(Rx_data[8] == '3') // làm nũng
		{
			StateMachine =6;
		}
		else if(Rx_data[8] == '4')
		{
			StateMachine =7;
		}
		else     // đứng im
		{
			StateMachine =0;
		}
	}
	if(Rx_data[10] == '1')  // thẳng
	{
		StateMachine =7;
	}
	else
	{
		HAL_GPIO_WritePin(COI_GPIO_Port, COI_Pin, 0);
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	/* Tần số 50Hz ( ngược kim đồng hồ )
	 * 0.5ms -> 0 độ
	 * 1.5ms -> 90 độ
	 * 2.5ms -> 180 độ
	 *
	 * Qua BJT nên đảo mức lại
	 *
	 * 20-0.5ms  -> 20ms
	 *  x        -> 1000
	 *  => 0 độ 975, 90 độ 925, 180 độ 875
	 *
	 *  */
  	HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 12); // Nhận Data



	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

//  SÉT GÓC BAN �?ẦU
	CHANGE_TOP_RIGHT_TREN(TopRightTren);   // MAX = 870,  MIN = 930;
	CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);   // MAX = 875, MIN = 912
	CHANGE_BOT_RIGHT_TREN(BotRightTren);   // MAX = 895, MIN = 960;
	CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);   // MAX = 912, MIN =  960;

	CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);     // MAX = 960, MIN = 912;
	CHANGE_TOP_LEFT_TREN(TopLeftTren);     // MAX = 960, MIN = 890;
	CHANGE_BOT_LEFT_TREN(BotLeftTren);	   // MAX = 940, MIN = 875;
	CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);     // MAX = 960, MIN = 912;

	HAL_Delay(1000);

	//	TODO
	TopRightTren = 875;
	TopRightDuoi = 896;  // 904
	BotRightTren = 960; // 950
	BotRightDuoi = 896;  // 904
	TopLeftTren = 930;  // 930
	TopLeftDuoi = 960;  // 920
	BotLeftTren = 915;
	BotLeftDuoi = 930; // 920


	CHANGE_TOP_RIGHT_TREN(TopRightTren);
	CHANGE_BOT_RIGHT_TREN(BotRightTren);
	CHANGE_TOP_LEFT_TREN(TopLeftTren);
	CHANGE_BOT_LEFT_TREN(BotLeftTren);


	CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
	CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
	CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
	CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);

	HAL_Delay(1000);

	TopRightTren = 915;
	BotRightTren = 920;  // 930
	TopLeftTren = 960; // 950
	BotLeftTren = 880;
	CHANGE_TOP_RIGHT_TREN(TopRightTren);
	CHANGE_BOT_RIGHT_TREN(BotRightTren);
	CHANGE_TOP_LEFT_TREN(TopLeftTren);
	CHANGE_BOT_LEFT_TREN(BotLeftTren);


	HAL_Delay(1000);

	LastTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// TODO
		if(DieuKhien == 1)
		{
			LastTime = HAL_GetTick();
		}
		else
		{
			if(HAL_GetTick()- LastTime > 7000)
			{
				StateMachine ++;
				if(StateMachine == 8) StateMachine =1;
				LastTime = HAL_GetTick();
			}
		}


		switch (StateMachine)
		{
			case 0:
			{
				DogBalance();
				break;
			}
			case 1:
			{
				if(State != 1)
				{
					HAL_GPIO_WritePin(COI_GPIO_Port, COI_Pin, 0);
					DogBalance();
					State =1;
					TopRightDuoi = 896;
					BotRightDuoi = 896;
					TopLeftDuoi = 960;
					BotLeftDuoi = 930;
					TopRightTren = 915;
					BotRightTren = 920;  // 930
					TopLeftTren = 960; // 950
					BotLeftTren = 880;

					CHANGE_TOP_RIGHT_TREN(TopRightTren);
					CHANGE_BOT_RIGHT_TREN(BotRightTren);
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
					CHANGE_BOT_LEFT_TREN(BotLeftTren);
					CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
					CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
					CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);
					CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
				}

				DogForward();
				break;
			}
			case 2:
			{
				if(State != 2)
				{
					DogBalance();
					State =2;

					BotRightTren = 920;
					TopLeftTren = 920;
					TopRightTren = 910;
					BotLeftTren = 910;

					CHANGE_TOP_RIGHT_TREN(TopRightTren);
					CHANGE_BOT_RIGHT_TREN(BotRightTren);
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
					CHANGE_BOT_LEFT_TREN(BotLeftTren);

				}
				DogTurnLeft();
				break;
			}
			case 3:
			{
				if(State != 3)
				{
					DogBalance();
					State =3;
					BotRightTren = 960;
					TopLeftTren = 960;
					TopRightTren = 870;
					BotLeftTren = 870;

					CHANGE_TOP_RIGHT_TREN(TopRightTren);
					CHANGE_BOT_RIGHT_TREN(BotRightTren);
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
					CHANGE_BOT_LEFT_TREN(BotLeftTren);
				}
				DogTurnRight();
				break;
			}
			case 4:
			{
				if(State != 4)
				{
					DogBalance();
					State =4;
					BotLeftDuoi = 930;
					CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
					TopRightDuoi = 875;
					CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
					TopLeftTren = 940;
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
					TopRightTren = 920;
					CHANGE_TOP_RIGHT_TREN(TopRightTren);

				}
				DogHello();
				break;
			}
			case 5:
			{
				if(State != 5)
				{
					DogBalance();
					State =5;
					BotRightTren= 960;
					CHANGE_BOT_RIGHT_TREN(BotRightTren);
					BotLeftTren = 880;
					CHANGE_BOT_LEFT_TREN(BotLeftTren);
					BotRightDuoi = 890;
					CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
					BotLeftDuoi = 930;
					CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);
					TopRightTren = 900;
					CHANGE_TOP_RIGHT_TREN(TopRightTren);
					TopLeftTren = 930;
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
				}
				DogHitDat();
				break;
			}
			case 6:
			{
				if(State != 6)
				{
					DogBalance();
					State =6;

					BotRightTren= 960;
					CHANGE_BOT_RIGHT_TREN(BotRightTren);
					BotLeftTren = 880;
					CHANGE_BOT_LEFT_TREN(BotLeftTren);
					BotRightDuoi = 890;
					CHANGE_BOT_RIGHT_DUOI(BotRightDuoi);
					BotLeftDuoi = 935;
					CHANGE_BOT_LEFT_DUOI(BotLeftDuoi);

					TopRightTren = 880;
					CHANGE_TOP_RIGHT_TREN(TopRightTren);
					TopLeftTren = 950;
					CHANGE_TOP_LEFT_TREN(TopLeftTren);
					TopRightDuoi = 875;
					CHANGE_TOP_RIGHT_DUOI(TopRightDuoi);
					TopLeftDuoi = 960;
					CHANGE_TOP_LEFT_DUOI(TopLeftDuoi);

				}
				DogDapChan();
				break;
			}
			case 7:
			{
				DogKeu();
				break;
			}
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1439;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 1439;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|COI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin COI_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|COI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
