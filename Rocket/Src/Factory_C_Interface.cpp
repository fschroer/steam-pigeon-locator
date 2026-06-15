#include <Factory.hpp>
#include <Factory_C_Interface.h>
#include "SpiBus.hpp"
#include "ITM_Trace.hpp"          // new shared helper
#include <string.h>             // for memcpy

extern "C" {
#include "tim.h"    // exposes TIM2 base-address macro and htim2
}

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

// Single owner of SPI2 (baro + IMU + flash).  Declared in SpiBus.hpp.
namespace RocketNav {
	SpiBus& Spi2Bus() { static SpiBus bus; return bus; }
}

extern "C" void RocketFactory_Init(const struct Radio_s* radio) {
	RocketNav::Spi2Bus().init(&hspi2);   // before sensors/timers come up
	ITM_Init();
    // Send a known byte to port 0 immediately after init
    // Loop to make it easy to catch
    for (int i = 0; i < 100; i++) {
        ITM_Trace::send(0, (uint32_t)'X');
    }
	factory.Init(radio);
}

// ---------------------------------------------------------------------------
// Timing-capture state — file-static, shared between the two callbacks below.
//
// OCCallback fires TWICE per 50 ms cycle:
//   call 1 (s_oc_first=true)  → starts D2 conversion (~28 ms into cycle)
//   call 2 (s_oc_first=false) → reads D2, starts D1   (~39 ms into cycle)
//
// ProcessRocketEvents fires once per cycle from the main loop.
// process_dur_us is the PREVIOUS cycle's duration (the current cycle's end
// time is not available when WriteData is called inside UpdateFlightState).
// ---------------------------------------------------------------------------
static bool     s_oc_first       = true;
static uint16_t s_oc_start_us    = 0;
static uint16_t s_oc_end_us      = 0;
static uint16_t s_process_dur_us = 0;

extern "C" void RocketFactory_ProcessRocketEvents(uint8_t rocket_service_count) {
    const uint16_t proc_start = static_cast<uint16_t>(TIM2->CNT);

    TimingDiag diag;
    diag.oc_start_us      = s_oc_start_us;
    diag.oc_end_us        = s_oc_end_us;
    diag.process_start_us = proc_start;
    diag.process_dur_us   = s_process_dur_us;   // previous cycle's duration

    factory.SetTimingDiag(diag);
    factory.ProcessRocketEvents(rocket_service_count);

    s_process_dur_us = static_cast<uint16_t>(static_cast<uint16_t>(TIM2->CNT) - proc_start);
}

extern "C" void RocketFactory_ServiceBus() {
	factory.ServiceBus();
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
    const uint16_t t_entry = static_cast<uint16_t>(TIM2->CNT);

    if (s_oc_first) {
        // First call in this 50 ms cycle: capture OC start time.
        s_oc_start_us = t_entry;
    }

    factory.MS5611OCCallback();

    if (s_oc_first) {
        s_oc_first = false;
    } else {
        // Second call: capture OC end time and reset toggle for next cycle.
        s_oc_end_us = static_cast<uint16_t>(TIM2->CNT);
        s_oc_first  = true;
    }

	ITM_Trace::send(2, (uint32_t)RocketNav::d_d2_converting_ms_);
	ITM_Trace::send(3, (uint32_t)RocketNav::d_d1_converting_ms_);
	ITM_Trace::send(4, (uint32_t)RocketNav::d_d1_converted_ms_);
}
