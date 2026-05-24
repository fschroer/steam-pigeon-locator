#ifdef __cplusplus
extern "C" {
#endif

//#include "stdint.h"
//
struct Radio_s;   // forward declaration only

void RocketFactory_Init(const struct Radio_s* radio);
void RocketFactory_ProcessRocketEvents(uint8_t rocket_service_count);
void RocketFactory_OnRadioTxDone();
void RocketFactory_OnRadioRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
void RocketFactory_ProcessUART2Char(uint8_t uart_char);
void RocketFactory_MS5611OCCallback();

#ifdef __cplusplus
}
#endif
