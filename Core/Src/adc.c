/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;

/* ADC init function */
void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  /* NOTE for whoever regenerates this file from Locator.ioc:
     TriggerFrequencyMode above must stay ADC_TRIGGER_FREQ_LOW.  The battery read
     enables the ADC, takes one conversion, and fully disables it again (see
     PowerManagement::readRawADCChecked) roughly once per second, which exceeds
     the datasheet "tIdle" between triggers.  ST's note on the parameter in
     stm32wlxx_hal_adc.h says low-frequency mode is required in exactly that
     case; the cost is 2 ADC clock cycles of rearm, ~83 ns here.  CubeMX
     defaults this to HIGH, so ADC.TriggerFrequencyMode was added to the .ioc's
     IPParameters list.

     That mechanism is verified, not assumed: on 2026-08-08 the line was set to
     HIGH by hand and the project regenerated (CubeIDE 1.19.0), and CubeMX wrote
     LOW back over it.  Note CubeMX only rewrites this file when its model
     differs from what is on disk, so a regeneration that leaves adc.c untouched
     is the setting already being correct, not the setting being skipped. */

  /* ADC self-calibration.  Prerequisite is that the ADC be disabled, which is
     the state this function leaves it in.  The resulting CALFACT survives the
     enable/disable cycle HAL_ADC_Stop performs on every battery read — the
     factor is lost only when the ADC voltage regulator is turned off, and that
     happens solely in HAL_ADC_DeInit.  So calibrating once here holds.

     Without it CALFACT stays 0 and every conversion carries the die's raw
     offset.  Measured on a locator on 2026-08-08: ~83 counts (~67 mV), which
     presented as a grounded BATTLVL node reading 63 mV and a VREFINT-derived
     VDDA of 3129 mV instead of ~3.29 V.  See docs/bench-battery-diag.md.

     Deliberately NOT routed to Error_Handler(): an uncalibrated ADC costs tens
     of millivolts on a battery gauge, while halting here would cost the whole
     locator, including the recovery beacon.  A failure is visible instead in
     the 'v' console diagnostic, which prints CALFACT and warns when it is 0. */
  (void) HAL_ADCEx_Calibration_Start(&hadc);
  /* USER CODE END ADC_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC)
  {
  /* USER CODE BEGIN ADC_MspInit 0 */

  /* USER CODE END ADC_MspInit 0 */
    /* ADC clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC GPIO Configuration
    PB4     ------> ADC_IN3
    */
    GPIO_InitStruct.Pin = BATTLVL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BATTLVL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC_MspInit 1 */

  /* USER CODE END ADC_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC)
  {
  /* USER CODE BEGIN ADC_MspDeInit 0 */

  /* USER CODE END ADC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC GPIO Configuration
    PB4     ------> ADC_IN3
    */
    HAL_GPIO_DeInit(BATTLVL_GPIO_Port, BATTLVL_Pin);

  /* USER CODE BEGIN ADC_MspDeInit 1 */

  /* USER CODE END ADC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

