#include <Factory.hpp>
#include <Factory_C_Interface.h>
#include "ITM_Trace.hpp"          // new shared helper
#include <string.h>             // for memcpy

// HAL handles provided by CubeMX (C symbols)
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc;
extern TIM_HandleTypeDef htim17;

namespace RocketNav {
	extern volatile uint32_t d_d2_converting_ms_;
	extern volatile uint32_t d_d1_converting_ms_;
	extern volatile uint32_t d_d1_converted_ms_;
}

#define SWO_SPEED 2000000UL    // 2 MHz — must match CubeIDE debug config exactly

static void ITM_Init(void) {
    // CoreDebug and ITM only — do NOT touch TPI registers
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = ITM_TCR_ITMENA_Msk   |
               ITM_TCR_SYNCENA_Msk  |
               ITM_TCR_SWOENA_Msk   |
               (1UL << ITM_TCR_TraceBusID_Pos);
    ITM->TER = 0xFFFFFFFF;
    ITM->TPR = 0xFFFFFFFF;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // TPI->ACPR, TPI->SPPR, TPI->FFCR intentionally omitted
    // ST-LINK GDB server configures these during session startup
}

// Single Factory instance
static Factory factory(huart2, hspi2, hi2c2, hadc, htim17);

extern "C" void RocketFactory_Init(const struct Radio_s* radio) {
	ITM_Init();
    // Send a known byte to port 0 immediately after init
    // Loop to make it easy to catch
    for (int i = 0; i < 100; i++) {
        ITM_Trace::send(0, (uint32_t)'X');
    }
	factory.Init(radio);
}

extern "C" void RocketFactory_ProcessRocketEvents(uint8_t rocket_service_count) {
	factory.ProcessRocketEvents(rocket_service_count);
//	ITM_Trace::send(ITM_Trace::SENSOR, device->sensorReading);
//	ITM_Trace::send(ITM_Trace::FILTER, device->filteredOutput);
}

extern "C" void RocketFactory_OnRadioTxDone() {
	factory.OnRadioTxDone();
}

extern "C" void RocketFactory_OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
	factory.OnRadioRxDone(payload, size, rssi, LoraSnr_FskCfo);
}

extern "C" void RocketFactory_ProcessUART2Char(uint8_t uart_char) {
	factory.ProcessUART2Char(uart_char);
}

extern "C" void RocketFactory_MS5611OCCallback() {
	factory.MS5611OCCallback();
	ITM_Trace::send(2, (uint32_t)RocketNav::d_d2_converting_ms_);
	ITM_Trace::send(3, (uint32_t)RocketNav::d_d1_converting_ms_);
	ITM_Trace::send(4, (uint32_t)RocketNav::d_d1_converted_ms_);
}
