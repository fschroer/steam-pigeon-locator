/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "app_subghz_phy.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdio.h>
#include "stm32wlxx_ll_usart.h"
//#include "stm32wlxx_ll_gpio.h"
#include "Constants.h"

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
const char* lora_startup_message_ = "Rocket Locator v1.3.1\r\n\0";
volatile uint32_t processRocketEventsInterruptCount = 0;

/* --- PPS timing state --- */
volatile static uint32_t pps_last_ts = 0;    // TIM2 timestamp of previous PPS
volatile static uint32_t pps_this_ts = 0;    // TIM2 timestamp of current PPS
volatile static int32_t  pps_period_err = 0; // Period error in microseconds
volatile static bool pps_update_pending = false;
volatile static bool arr_update_pending = false;
volatile static uint32_t elapsed = 0;
volatile static uint32_t tim17_arr = 49999;
volatile static uint32_t pps_count = 0;
volatile static uint8_t rocket_service_count = 0;
volatile static uint32_t tim17_cnt_pps = 0;
volatile uint32_t processTime = 0;

//extern void RocketFactory_Init(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_SubGHz_Phy_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  MX_USART2_UART_Init();
  LL_USART_EnableIT_RXNE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
  RocketFactory_Init(&Radio);

  tim17_arr = htim17.Init.Period;
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim17);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
  	if (processRocketEventsInterruptCount > 0) {
  		processRocketEventsInterruptCount--;
  		rocket_service_count++;
  		if (rocket_service_count == SAMPLES_PER_SECOND)
				rocket_service_count = 0;
  		uint32_t start = TIM2->CNT;
      RocketFactory_ProcessRocketEvents();
  		uint32_t end = TIM2->CNT;
  		processTime = end - start;
  	}

    if (pps_update_pending) {
        pps_update_pending = false;
//				printf("%.4lu Elapsed=%lu ARR:%lu, TIM17CNT:%lu\n"
//						, pps_count, elapsed, tim17_arr, tim17_cnt_pps);
    }
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LoraTxCallback() {
  RocketFactory_OnRadioTxDone();
}

void LoraRxCallback(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	RocketFactory_OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo);
}

void UART2_CharReception_Callback(void) {
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
	RocketFactory_ProcessUART2Char(LL_USART_ReceiveData8(USART2));
}

void UART_TXEmpty_Callback(void) {
}

void UART_CharTransmitComplete_Callback(void) {
}

void UART_Error_Callback(void) {
  //__IO uint32_t isr_reg;

  // Disable USARTx_IRQn
  /*NVIC_DisableIRQ(USART1_IRQn);

  //Error handling example :
  //  - Read USART ISR register to identify flag that leads to IT raising
  //  - Perform corresponding error handling treatment according to flag

  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    // Turn LED3 on: Transfer error in reception/transmission process
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // LED_RED
  }
  else
  {
    // Turn LED3 on: Transfer error in reception/transmission process
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); // LED_RED
  }*/
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) ||
      ((huart->ErrorCode & (HAL_UART_ERROR_RTO | HAL_UART_ERROR_ORE)) != 0U))
  {
	  if (HAL_UART_Init(huart) != HAL_OK){
	    Error_Handler();
	  }
	  LL_USART_EnableIT_RXNE(huart->Instance);
		LL_USART_EnableIT_ERROR(huart->Instance);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM17) {
    	processRocketEventsInterruptCount++;
    	if (arr_update_pending) {
    		TIM17->ARR = tim17_arr;
    		arr_update_pending = false;
    	}
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_6)	{
    tim17_cnt_pps = TIM17->CNT;
		pps_this_ts = TIM2->CNT;
		elapsed = pps_this_ts - pps_last_ts;
		pps_period_err = elapsed - 1000000;
		pps_last_ts = pps_this_ts;

    if (pps_period_err > -200 && pps_period_err < 200)
        return;
    tim17_arr = elapsed / SAMPLES_PER_SECOND - 1;
    arr_update_pending = true;
		pps_update_pending = true;
		pps_count++;
	}
}

//int _write(int file, char *ptr, int len) {
//    for (int i = 0; i < len; i++) {
//        while (!LL_USART_IsActiveFlag_TXE(USART2));
//        LL_USART_TransmitData8(USART2, ptr[i]);
//    }
//    while (!LL_USART_IsActiveFlag_TC(USART2));
//    return len;
//}

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
#ifdef USE_FULL_ASSERT
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
