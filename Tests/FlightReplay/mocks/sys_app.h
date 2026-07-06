/* Host mock for CubeMX sys_app.h — only what FlightManager needs. */
#pragma once
#include <stdint.h>

/* The real PowerManagement.hpp (Rocket/Inc, same dir as FlightManager.hpp, so
   it wins quote-include over -Imocks) references ADC_HandleTypeDef.  Provide a
   host stand-in so it parses; FlightManager only stores the reference, never
   calls into it, and the constructor is stubbed in the harness TU. */
typedef struct ADC_HandleTypeDef_s { int _unused; } ADC_HandleTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

/* Monotonic millisecond tick. Backed by the harness bench clock (MockEnv.cpp). */
uint32_t HAL_GetTick(void);

#ifdef __cplusplus
}
#endif
