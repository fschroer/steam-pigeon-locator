#pragma once

extern "C" {
#include "gpio.h"
}

enum class RgbColor : uint8_t {
	Off = 0,
	Red,
	Green,
	Blue,
	Cyan,
	Magenta,
	Yellow
};

inline void RgbLed(RgbColor color) {
	switch (color) {
		case RgbColor::Off :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
		case RgbColor::Red :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
		case RgbColor::Green :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
		case RgbColor::Blue :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_RESET);
			break;
		case RgbColor::Cyan :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_RESET);
			break;
		case RgbColor::Magenta :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_RESET);
			break;
		case RgbColor::Yellow :
			HAL_GPIO_WritePin(SOFT_LED1_GPIO_Port, SOFT_LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED2_GPIO_Port, SOFT_LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SOFT_LED3_GPIO_Port, SOFT_LED3_Pin, GPIO_PIN_SET);
			break;
	}
};
