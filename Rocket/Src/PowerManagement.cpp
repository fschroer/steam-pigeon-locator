extern "C" {
#include "adc.h"
#include "gpio.h"
}

#include <PowerManagement.hpp>

// ADC constants
static constexpr uint32_t ADC_REF_mV      = 3300;     // millivolts
static constexpr uint32_t ADC_MAX_COUNTS  = 4095;

// Divider inverse ratio
static constexpr uint32_t DIVIDER_TOTAL_RESISTANCE    = 34500;
static constexpr uint32_t DIVIDER_MEASURED_RESISTANCE = 27000;

PowerManagement::PowerManagement(ADC_HandleTypeDef* hadc)
	: m_hadc(hadc) {
}

void PowerManagement::enableDivider() {
	HAL_GPIO_WritePin(BATTRD_GPIO_Port, BATTRD_Pin, GPIO_PIN_SET);
}

void PowerManagement::disableDivider() {
	HAL_GPIO_WritePin(BATTRD_GPIO_Port, BATTRD_Pin, GPIO_PIN_RESET);
}

uint16_t PowerManagement::readRawADC() {
//	uint32_t start = TIM2->CNT;
	HAL_ADC_Start(m_hadc);
	HAL_ADC_PollForConversion(m_hadc, 10);
	uint16_t value = HAL_ADC_GetValue(m_hadc);
	HAL_ADC_Stop(m_hadc);
//	uint32_t elapsed = TIM2->CNT - start;
	return value;
}

uint16_t PowerManagement::convertToMillivolts(uint16_t raw) {
	// Convert ADC reading to millivolts
	uint32_t v_adc_mV = (raw * ADC_REF_mV) / ADC_MAX_COUNTS;

	// Undo the divider using scaled integer math
	uint32_t v_batt_mV = (v_adc_mV * DIVIDER_TOTAL_RESISTANCE) / DIVIDER_MEASURED_RESISTANCE;

	return static_cast<uint16_t>(v_batt_mV);
}

uint16_t PowerManagement::readBatteryMillivolts() {
//	enableDivider();

//	HAL_Delay(25);   // allow divider to settle
	uint16_t raw = readRawADC();

	disableDivider();

	return convertToMillivolts(raw);
}
