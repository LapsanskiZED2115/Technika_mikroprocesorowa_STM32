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
#include <stdio.h>
#define buff_length 100
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int liczba=0;
static uint8_t licznik_przerwan=0;
uint8_t tysiac, setka, dziesiatka, jednostka;
uint32_t current_time, last_but_press_time;
const int stala_FFFF = 0xFFFFFFFF;
int suma;
volatile uint16_t cnt, buf_cnt, ADC_flag_Ready;
volatile uint16_t buf_filtra[1050];
volatile uint8_t buf_tx[100];
volatile uint8_t buf_rx[100];
volatile uint16_t Mems[3];

volatile uint16_t ADCValue[3];

//volatile uint8_t TxBuff[buff_length] = {48, 49, 50, 97};
volatile uint8_t TxBuff[buff_length];



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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void przelicz(uint16_t liczba)
{
	if (liczba < 10 && 0<liczba)
	      {jednostka=liczba;
	      }
	      else if (9 <liczba &&liczba < 100)
	      {
	          dziesiatka = liczba / 10;
	          jednostka = liczba % 10;

	      }
	      else if (99<liczba && liczba < 1000)
	      {
	          // Extract hundreds, tens, and units
	          setka = liczba / 100;
	          dziesiatka = (liczba / 10) % 10;
	          jednostka = liczba % 10;

	      }
	      else if(999< liczba && liczba < 9999)
	      {
	          // Extract thousands, hundreds, tens, and units
	          tysiac = liczba / 1000;
	          setka = (liczba / 100) % 10;
	          dziesiatka = (liczba / 10) % 10;
	          jednostka = liczba % 10;

	      }

}
void wypisz_liczbe(uint8_t cyfra)
{
      HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin | Dp_Pin, GPIO_PIN_RESET);

      switch (cyfra)
      {
          case 0:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin, GPIO_PIN_SET);
              break;
          case 1:
              HAL_GPIO_WritePin(GPIOC, B_Pin | C_Pin, GPIO_PIN_SET);
              break;
          case 2:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | D_Pin | E_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 3:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 4:
              HAL_GPIO_WritePin(GPIOC, B_Pin | C_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 5:
              HAL_GPIO_WritePin(GPIOC, A_Pin | C_Pin | D_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 6:
              HAL_GPIO_WritePin(GPIOC, A_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 7:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin, GPIO_PIN_SET);
              break;
          case 8:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
              break;
          case 9:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | F_Pin | G_Pin, GPIO_PIN_SET);
              break;
          default:
              HAL_GPIO_WritePin(GPIOC, A_Pin | B_Pin | C_Pin | D_Pin | E_Pin | F_Pin | G_Pin | Dp_Pin, GPIO_PIN_RESET);
              break;
      }
  }

void wyb_com(uint8_t com)
  {
      HAL_GPIO_WritePin(GPIOC, COM1_Pin | COM2_Pin | COM3_Pin | COM4_Pin, GPIO_PIN_SET);

      switch (com)
      {
          case 3:
              HAL_GPIO_WritePin(GPIOC, COM1_Pin, GPIO_PIN_RESET);
              break;
          case 2:
              HAL_GPIO_WritePin(GPIOC, COM2_Pin, GPIO_PIN_RESET);
              break;
          case 1:
              HAL_GPIO_WritePin(GPIOC, COM3_Pin, GPIO_PIN_RESET);
              break;
          case 0:
              HAL_GPIO_WritePin(GPIOC, COM4_Pin, GPIO_PIN_RESET);
              break;
          default:
              return;
      }


  }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM11){
		if (licznik_przerwan>3)
		{
			licznik_przerwan=0;
		}

		if(licznik_przerwan==0)
				{
			wyb_com(0);
				wypisz_liczbe(jednostka);

				}
		else if(licznik_przerwan==1)
				{
			wyb_com(1);
				wypisz_liczbe(dziesiatka);

				}
		else if(licznik_przerwan==2)
				{
			wyb_com(2);
				wypisz_liczbe(setka);

				}
		else if(licznik_przerwan==3){
			wyb_com(3);
			wypisz_liczbe(tysiac);}
		licznik_przerwan=licznik_przerwan+1;

	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//staticHAL_ADC_ConvCpltCallback
	if(GPIO_Pin == sw1_Pin)
	{
        current_time=HAL_GetTick();

        	if((current_time- last_but_press_time)>200 && (stala_FFFF -1)- current_time + last_but_press_time>500)
        	{
        		   liczba++;
        		   przelicz(liczba);
        	last_but_press_time=current_time;
		   }

       }

	if(GPIO_Pin == sw2_Pin)
		{
	        current_time=HAL_GetTick();
	        	if((current_time- last_but_press_time)>200 && (stala_FFFF -1)- current_time + last_but_press_time>500)
	        	{
	        		   liczba--;
	        		   przelicz(liczba);
	        	last_but_press_time=current_time;
			   }
	       }
}


//
//HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//
//	//
// HAL_UART_Receive_IT(huart, (uint8_t*)buf_rx, 1);
// if (buf_rx[0]>96 && buf_rx[0]<123)
//		 {
//	 buf_cnt = snprintf((char*)buf_tx,sizeof(buf_tx) ,"Napis nr= %d, ASCII=%d, litera=%c \n", cnt, buf_rx[0], (char)buf_rx[0]);
//		 }
// else
// {
//	 buf_cnt = snprintf((char*)buf_tx,sizeof(buf_tx) ,"Podales liczbe, pod ktora nie kryje sie mala litera, %d\n", buf_rx[0]);
//
// }
// HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf_tx, buf_cnt);
// HAL_UART_Receive_IT(huart, (uint8_t*)buf_rx, 1);
//
//}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	ADC_flag_Ready=1;

//	if(HAL_ADC_PollForConversion(&hadc1, 10)==HAL_OK)
//	{
//		if(hadc==&hadc1)
//		{
//
//			if(buf_cnt==1023)
//			{
//				przelicz(suma>>10);
//				suma=0;buf_cnt=0;
//			}
//		suma+=HAL_ADC_GetValue(&hadc1);
//
//		buf_cnt++;
//		}}

}



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
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim11);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//buf_cnt = sprintf((char*) buf_tx, "Napis nr= %d\n", cnt);
//HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf_tx, buf_cnt);

//HAL_UART_Receive_IT(&huart2, (uint8_t*)buf_rx, 1);
//HAL_ADC_Start_IT(&hadc1);
//	ADCValue=HAL_ADC_GetValue(&hadc1);
//
//HAL_TIM_Base_Start(&htim2);
//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCValue, 2);
HAL_TIM_Base_Start(&htim2);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCValue, 3);

uint8_t MEMSTxBuf[2];
  uint8_t MEMSRxBuf[2];

  MEMSTxBuf[0] = 0x00 | 0x20;
  MEMSTxBuf[1] = 0x47;

  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, MEMSTxBuf, 2, 10);
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

//stat_var & 0x08) != 0
  while (1)
  {
	  uint8_t status;



			MEMSRxBuf[0] = 0x80 | 0x27;
			MEMSRxBuf[1] = 0x29;
			HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
			HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
			status = MEMSRxBuf[1];

			if (status & 0x08 != 0)
			{
				MEMSRxBuf[0] = 0x80 | 0x29;
				MEMSRxBuf[1] = 0x29;

				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
				Mems[0] = MEMSRxBuf[1];

				MEMSRxBuf[0] = 0x80 | 0x2B;
				MEMSRxBuf[1] = 0x29;

				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
				Mems[1] = MEMSRxBuf[1];

				MEMSRxBuf[0] = 0x80 | 0x2D;
				MEMSRxBuf[1] = 0x29;

				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
				HAL_SPI_Receive(&hspi1, MEMSRxBuf, 2, 10);
				HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
				Mems[2] = MEMSRxBuf[1];


			    uint16_t TxSize = snprintf(buf_tx, sizeof(buf_tx), "X: %d, Y: %d, Z: %d\n", Mems[0], Mems[1], Mems[2]);
			    HAL_UART_Transmit_DMA(&huart2, buf_tx, TxSize);
		  			    HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 400;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 499;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_Pin|D_Pin|E_Pin|C_Pin
                          |B_Pin|F_Pin|A_Pin|Dp_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : G_Pin D_Pin E_Pin C_Pin
                           B_Pin F_Pin A_Pin Dp_Pin
                           COM4_Pin COM3_Pin COM2_Pin COM1_Pin */
  GPIO_InitStruct.Pin = G_Pin|D_Pin|E_Pin|C_Pin
                          |B_Pin|F_Pin|A_Pin|Dp_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : sw1_Pin sw2_Pin */
  GPIO_InitStruct.Pin = sw1_Pin|sw2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
