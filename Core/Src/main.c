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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
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

/* USER CODE BEGIN PV */
/* adc vars */
uint16_t ADC_Raw[1];
/* ntc vars */
float Ntc_Tmp = 0;
uint16_t Ntc_R;
/* sheduler vars */
uint8_t Sch_100ms = 255;

/* R1 resistance */
#define NTC_UP_R 10000.0f
/* constants of Steinhart-Hart equation */
#define A 0.001111f
#define B 0.000237987f
#define C 0.000000065f
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define FILTER_SMA_ORDER 12
uint16_t Filter_Buffer[FILTER_SMA_ORDER] = { 0 };
/**
 * @brief Simple Moving Average (SMA) filter.
 * @note Before use define filter order.
 * @param[in] Input raw (unfiltered) value.
 * @retval Return filtered data.
 */
uint16_t Filter_SMA(uint16_t For_Filtered);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Simple Moving Average (SMA) filter.
 * @note Before use define filter order.
 * @param[in] Input raw (unfiltered) value.
 * @retval Return filtered data.
 */
uint16_t Filter_SMA(uint16_t For_Filtered)
{
    /* Load new value */
    Filter_Buffer[FILTER_SMA_ORDER - 1] = For_Filtered;
    /* For output value */
    uint32_t Output = 0;
    /* Sum */
    for (uint8_t i = 0; i < FILTER_SMA_ORDER; i++) {
        Output += Filter_Buffer[i];
    }
    /* Divide */
    Output /= FILTER_SMA_ORDER;
    /* Left Shift */
    for (uint8_t i = 0; i < FILTER_SMA_ORDER; i++) {
        Filter_Buffer[i] = Filter_Buffer[i + 1];
    }
    /* Return filtered value */
    return (uint16_t) Output;
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    /* adc calibration */
    HAL_ADCEx_Calibration_Start(&hadc);
    /* adc dma start */
    HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADC_Raw, 1);
    /* timer 3 (f=1Hz, T=1000ms start */
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (Sch_100ms) {
            /* get adc value */
            HAL_ADC_Start_IT(&hadc);
            /* filtering (sma) */
            uint16_t Ntc_Temp_ADC = Filter_SMA(ADC_Raw[0]);
            /* calc. ntc resistance */
            Ntc_R = ((NTC_UP_R) / ((4095.0 / Ntc_Temp_ADC) - 1));
            /* temp */
            float Ntc_Ln = log(Ntc_R);
            /* calc. temperature */
            Ntc_Tmp = (1.0 / (A + B * Ntc_Ln + C * Ntc_Ln * Ntc_Ln * Ntc_Ln))
                    - 273.15;
            /* nullify */
            Sch_100ms = 0;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM3) {
        /* set every 100ms */
        Sch_100ms = 255;
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
