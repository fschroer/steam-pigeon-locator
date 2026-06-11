/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.c
  * @author  MCD Application Team
  * @brief   Application of the SubGHz_Phy Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "main.h"

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_TIMEOUT_VALUE    3000
#define TX_TIMEOUT_VALUE    3000
#define MAX_APP_BUFFER_SIZE 255

#ifndef RF_FREQUENCY
#define RF_FREQUENCY                                902300000 /* Hz */
#else
#undef RF_FREQUENCY
#define RF_FREQUENCY                                902300000 /* Hz */
#endif /* RF_FREQUENCY */

#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER                             22        /* dBm */
#else
#undef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER                             22        /* dBm */
#endif /* TX_OUTPUT_POWER */

#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#else
#undef LORA_BANDWIDTH
#define LORA_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
#endif /* LORA_BANDWIDTH */

#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR                       7         /* [SF7..SF12] */
#else
#undef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR                       7         /* [SF7..SF12] */
#endif /* LORA_SPREADING_FACTOR */

#ifndef LORA_CODINGRATE
#define LORA_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#else
#undef LORA_CODINGRATE
#define LORA_CODINGRATE                             1         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
#endif /* LORA_CODINGRATE */

#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#else
#undef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#endif /* LORA_PREAMBLE_LENGTH */

#ifndef LORA_SYMBOL_TIMEOUT
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#else
#undef LORA_SYMBOL_TIMEOUT
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#endif /* LORA_SYMBOL_TIMEOUT */

#ifndef LORA_FIX_LENGTH_PAYLOAD_ON
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#else
#undef LORA_FIX_LENGTH_PAYLOAD_ON
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#endif /* LORA_FIX_LENGTH_PAYLOAD_ON */

#ifndef LORA_IQ_INVERSION_ON
#define LORA_IQ_INVERSION_ON                        false
#else
#undef LORA_IQ_INVERSION_ON
#define LORA_IQ_INVERSION_ON                        false
#endif /* LORA_IQ_INVERSION_ON */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
  Radio.SetChannel(902300000); //Saved config frequency set in RocketFactory.
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  Radio.Rx(RX_TIMEOUT_VALUE);

  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  LoraTxCallback();
  Radio.Rx(RX_TIMEOUT_VALUE);
  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  LoraRxCallback(payload, size, rssi, LoraSnr_FskCfo);
  Radio.Rx(RX_TIMEOUT_VALUE);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  Radio.Rx(RX_TIMEOUT_VALUE);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  Radio.Rx(RX_TIMEOUT_VALUE);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */
