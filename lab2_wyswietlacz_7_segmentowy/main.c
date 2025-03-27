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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int liczba=0;
static uint8_t licznik_przerwan=0;
uint8_t tysiac, setka, dziesiatka, jednostka;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
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
		else if(licznik_przerwan==3)
				{
			wyb_com(3);
				wypisz_liczbe(tysiac);

				}
		licznik_przerwan=licznik_przerwan+1;

	}
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
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim11);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
liczba=liczba+1;
przelicz(liczba);

HAL_Delay(10);

//	  	if(licznik_przerwan==0)
//	  					{
//	  				wypisz_liczbe(jednostka);
//	  					}
//	  			else if(licznik_przerwan==1)
//	  					{
//	  				wypisz_liczbe(dziesiatka);
//	  					}
//	  			else if(licznik_przerwan==2)
//	  					{
//	  				wypisz_liczbe(setka);
//
//	  					}
//	  			else if(licznik_przerwan==3)
//	  					{
//	  				wypisz_liczbe(tysiac);
//	  					}
	  //jedna funkcja od przedk
	  // druga od wyboru
	  // 3 id przeliczenia

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G_Pin|D_Pin|E_Pin|C_Pin
                          |B_Pin|F_Pin|A_Pin|Dp_Pin
                          |COM4_Pin|COM3_Pin|COM2_Pin|COM1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
