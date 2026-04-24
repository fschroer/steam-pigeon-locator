#include <Factory.hpp>
#include <Factory_C_Interface.h>

// HAL handles provided by CubeMX (C symbols)
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc;

// Single Factory instance
static Factory factory(huart2, hspi2, hi2c2, hadc);

extern "C" void RocketFactory_Init(const struct Radio_s* radio) {
	factory.Init(radio);
}

extern "C" void RocketFactory_ProcessRocketEvents(uint8_t rocket_service_count) {
	factory.ProcessRocketEvents(rocket_service_count);
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
