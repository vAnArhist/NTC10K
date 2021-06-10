/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include "cmsis_os.h"
#include "Logger.h"
#include "Uart.hpp"
#include "Ntc.hpp"

#include <stdio.h>

typedef StaticTask_t osStaticThreadDef_t;
osThreadId_t InitTaskHandle;
uint32_t InitTaskBuffer[1024];
osStaticThreadDef_t InitTaskControlBlock;
static float ntcTemp = 0.0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void StartInitTask(void* argument);

bool writeToLog(const uint8_t* data, uint32_t size)
{
    static Driver::Uart *pUart = nullptr;

    if (pUart == nullptr) {
        pUart = Driver::Uart::getInstance(Driver::Uart::UART_2);
    }

    bool rv = pUart->write(data, size);

    while (pUart->isBusyTx()) {
        // Wait for uart ready
    }

    return rv;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock to 80Mhz */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    /* Init scheduler */
    osKernelInitialize();

    osThreadAttr_t InitTask_attributes = { 0 };
    InitTask_attributes.name = "InitTask";
    InitTask_attributes.stack_mem = &InitTaskBuffer[0];
    InitTask_attributes.stack_size = sizeof(InitTaskBuffer);
    InitTask_attributes.cb_mem = &InitTaskControlBlock;
    InitTask_attributes.cb_size = sizeof(InitTaskControlBlock);
    InitTask_attributes.priority = (osPriority_t) osPriorityNormal;

    /* Create the thread(s) */
    /* creation of defaultTask */
    InitTaskHandle = osThreadNew(StartInitTask, NULL, &InitTask_attributes);

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    while (1) {

    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

    /* MSI is enabled after System reset, activate PLL with MSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLP = 7;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* Initialization Error */
        while (1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        /* Initialization Error */
        while (1);
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LD3_Pin */
    GPIO_InitStruct.Pin = LD3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartInitTask(void* argument)
{
    LOG_INIT(writeToLog);
    LOG_INFO_WP("Logger initialized!\n");
    Driver::Ntc *ntc = Driver::Ntc::getInstance();

    for (;;) {
        ntcTemp = ntc->getTemperature();
        LOG_INFO_WP("$Temperature: %2.2f;\n", ntcTemp);
        //LOG_INFO_WP("$Resistance: %d;\n", ntc->getResistance());
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        osDelay(100);
    }
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
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

#if 0
#define NEW_LOG_DEBUG(...)  LOG_DEBUG(__VA_ARGS__)
#else
#define NEW_LOG_DEBUG(...)
#endif
void* operator new(size_t sz)
{
    NEW_LOG_DEBUG("new %u Min: %u Free: %u\r\n", sz, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
    return pvPortMalloc(sz);
}

void* operator new[](size_t sz)
{
    NEW_LOG_DEBUG("new[] %u Min: %u Free: %u\r\n", sz, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
    return pvPortMalloc(sz);
}

void operator delete(void* p)
{
    vPortFree(p);
    NEW_LOG_DEBUG("delete 0x%08X. Min: %u Free: %u\r\n", p, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
}

void operator delete[](void* p)
{
    vPortFree(p);
    NEW_LOG_DEBUG("delete[] 0x%08X. Min: %u Free: %u\r\n", p, xPortGetMinimumEverFreeHeapSize(), xPortGetFreeHeapSize());
}
