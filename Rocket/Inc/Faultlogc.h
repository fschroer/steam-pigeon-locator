#pragma once
/* FaultLogC.h — plain C interface for the two FaultLog calls needed from main.c
 *
 * Include this header in main.c instead of FaultLog.hpp.
 * FaultLog.hpp remains included only from C++ translation units (Factory.cpp etc.).
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Call as the very first line of main(), before HAL_Init().
 * Snapshots RCC_CSR reset-cause bits before HAL_Init() clears them. */
void FaultLog_Init(void);

/* Call once per main loop iteration.  Refreshes the IWDG and records
 * checkpoint_tag (e.g. rocket_service_count) so a watchdog hang report
 * shows which phase the loop was in when it stopped. */
void FaultLog_KickWatchdog(unsigned int checkpoint_tag);

#ifdef __cplusplus
}
#endif
