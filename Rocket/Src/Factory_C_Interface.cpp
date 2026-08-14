#include <Factory.hpp>
#include <Factory_C_Interface.h>
#include "SpiBus.hpp"
#include "ITM_Trace.hpp"          // new shared helper
#include <string.h>             // for memcpy

extern "C" {
#include "tim.h"    // exposes TIM2 base-address macro and htim2
uint32_t Pps_GetTim2TicksPerSec(void);   // GPS-PPS-disciplined TIM2 ticks/sec (main.c)
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
// This file used to carry per-cycle TIM2 timestamps (oc_start / oc_end /
// process_start / process_dur) plus a first/second-call toggle, all of it feeding
// TimingDiag into the archived FlightSample.  ARCHIVE_VERSION 6 (#38) repurposed
// those four sample fields as raw GPS velocity and accuracy, so nothing read them
// any more and the whole capture path is gone.
//
// Nothing is lost operationally: the MS5611 D2/D1 sequence is owned by
// MS5611::OCCallback / ServiceConversions (its own m_state, with CC1 armed
// one-shot per cycle), and per-cycle timing is still measured live by
// CycleProfiler's Diag:: segment API — which is what the console 't' breakdown
// prints, and is independent of this path.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// GPS-PPS-disciplined monotonic millisecond clock.
//
// TIM2 free-runs at a nominal 1 MHz (prescaler 47) but its true rate drifts with
// the MSI oscillator.  Pps_GetTim2TicksPerSec() returns the TIM2 ticks measured
// between two GPS PPS edges (≈1e6; 0 until PPS lock), so dividing the per-cycle
// TIM2 delta by it yields real elapsed time anchored to GPS — the same timebase
// the NFR-9 strapdown dt uses.  Accumulated in microseconds (64-bit) and exposed
// as milliseconds; the 50 ms loop samples far inside the ~71-min 32-bit wrap, so
// the uint32 delta is wrap-safe.
// ---------------------------------------------------------------------------
static uint64_t s_mono_us     = 0;
static uint32_t s_prev_tim2   = 0;
static bool     s_mono_inited = false;

static uint32_t AdvanceMonotonicMs() {
    const uint32_t tim2_now = TIM2->CNT;
    if (s_mono_inited) {
        const uint32_t dt_ticks = tim2_now - s_prev_tim2;   // wrap-safe
        uint32_t tps = Pps_GetTim2TicksPerSec();
        if (tps == 0u)
            tps = 1000000u;                                  // nominal until PPS lock
        s_mono_us += static_cast<uint64_t>(dt_ticks) * 1000000ull / tps;
    } else {
        s_mono_inited = true;
    }
    s_prev_tim2 = tim2_now;
    return static_cast<uint32_t>(s_mono_us / 1000ull);
}

extern "C" void RocketFactory_ProcessRocketEvents(uint8_t rocket_service_count) {
    factory.SetFlightClockMs(AdvanceMonotonicMs());
    factory.ProcessRocketEvents(rocket_service_count);
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
    factory.MS5611OCCallback();

	ITM_Trace::send(2, (uint32_t)RocketNav::d_d2_converting_ms_);
	ITM_Trace::send(3, (uint32_t)RocketNav::d_d1_converting_ms_);
	ITM_Trace::send(4, (uint32_t)RocketNav::d_d1_converted_ms_);
}
