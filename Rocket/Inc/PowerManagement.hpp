#pragma once
extern "C" {
#include <cstdint>
}

class PowerManagement
{
public:
    explicit PowerManagement(ADC_HandleTypeDef* hadc);

    // Returns battery voltage in millivolts
    uint16_t readBatteryMillivolts();
    void enableDivider();
    void disableDivider();

private:
    ADC_HandleTypeDef* m_hadc;

    uint16_t readRawADC();
    uint16_t convertToMillivolts(uint16_t raw);
};
