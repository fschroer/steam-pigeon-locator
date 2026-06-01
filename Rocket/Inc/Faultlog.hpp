#pragma once
#include <stdint.h>

// =============================================================================
// FaultLog — persistent crash and hang diagnostics for STM32WL5
//
// WHAT IT CAPTURES
//   HardFault / BusFault / UsageFault / MemManage:
//     • Program Counter at the moment of fault (where the code was)
//     • Link Register (where it was called from)
//     • Full Cortex-M exception frame: R0-R3, R12, LR, PC, xPSR
//     • Stack Pointer value (to detect stack overflow)
//     • CFSR / HFSR / MMFAR / BFAR fault status registers
//
//   Watchdog expiry (hang detection):
//     • Last checkpoint tag written by the main loop before the hang
//     • Uptime at the last successful kick
//
//   Normal resets / power-on:
//     • Reset cause from RCC_CSR
//     • Uptime at reset
//
// HOW IT SURVIVES RESETS
//   The struct lives in a dedicated .noinit RAM section.  The startup
//   code zeroes .data and .bss but does NOT touch .noinit, so the struct
//   retains its contents across a reset.  A magic number distinguishes
//   a valid log from unprogrammed / power-on RAM.
//
// LINKER SCRIPT CHANGE REQUIRED
//   In STM32WL5MOCx_FLASH.ld, add a .noinit output section BEFORE .data:
//
//     .noinit (NOLOAD) :
//     {
//       . = ALIGN(4);
//       KEEP(*(.noinit))
//       . = ALIGN(4);
//     } >RAM
//
//   This tells the linker to place .noinit in RAM but never initialise it.
//   In STM32CubeIDE: open the .ld file, find the .data section, insert above.
// =============================================================================

namespace Diag {

// Fault type tag written before the CPU state is saved.
// Lets you distinguish a fault handler entry from a watchdog hang.
enum class FaultType : uint32_t {
    None        = 0,
    HardFault   = 0xFA01FA01,
    BusFault    = 0xFA02FA02,
    UsageFault  = 0xFA03FA03,
    MemManage   = 0xFA04FA04,
    WatchdogHang= 0xFA05FA05,
    AssertFail  = 0xFA06FA06,
};

// Full Cortex-M4 exception stack frame pushed automatically by the CPU
// plus extra registers saved by our fault handler prologue.
struct ExceptionFrame {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;       // return address (Link Register at fault entry)
    uint32_t pc;       // faulting instruction address
    uint32_t xpsr;
};

struct FaultRecord {
    static constexpr uint32_t kMagic = 0xDEAD1234;

    uint32_t      magic;           // kMagic when record is valid
    FaultType     fault_type;      // which handler fired, or Watchdog
    ExceptionFrame frame;          // CPU registers at fault
    uint32_t      sp;              // stack pointer (detect overflow vs. limit)
    uint32_t      cfsr;            // Configurable Fault Status Register
    uint32_t      hfsr;            // HardFault Status Register
    uint32_t      mmfar;           // MemManage Fault Address Register
    uint32_t      bfar;            // BusFault Address Register
    uint32_t      rcc_csr;         // Reset cause (copied on boot before RCC clears it)
    uint32_t      uptime_ms;       // HAL_GetTick() at fault / last watchdog kick
    uint32_t      watchdog_checkpoint; // last tag written by KickWatchdog()
    uint32_t      boot_count;      // increments every power-on; tracks restart loops
    char          assert_file[48]; // populated by FAULT_ASSERT macro
    uint32_t      assert_line;
};

// Placed in .noinit so it is NOT zeroed on reset.
// __attribute__((section(".noinit"))) is applied in FaultLog.cpp.
extern FaultRecord g_fault_log;

// ---------------------------------------------------------------------------
// API used from application code
// ---------------------------------------------------------------------------

// Call once at the very start of main(), before HAL_Init(), to:
//   • Save RCC_CSR reset-cause bits (cleared by HAL_Init)
//   • Increment boot_count if magic is valid, else initialise the record
void FaultLogInit();

// Returns true if the stored record contains a valid fault from a previous boot.
// Call after FaultLogInit() — typically in Factory::Init().
bool FaultLogHasRecord();

// Returns a pointer to the stored record (valid only if FaultLogHasRecord()).
const FaultRecord* FaultLogGet();

// Erase the stored record (call after you have successfully transmitted it).
void FaultLogClear();

// Watchdog checkpoint — call this in your main loop tick with a tag that
// identifies the current phase (e.g. the rocket_service_count value or a
// stage enum).  The IWDG is refreshed here.  If the loop hangs the watchdog
// fires, resets the CPU, and the last tag plus uptime are preserved in g_fault_log.
void KickWatchdog(uint32_t checkpoint_tag);

// Assertion helper — writes file/line then triggers a HardFault deliberately.
// Use the FAULT_ASSERT(cond) macro rather than calling this directly.
void FaultAssert(const char* file, uint32_t line);

} // namespace Diag

// Lightweight assert that fires the fault machinery when cond is false.
// Produces a FaultType::AssertFail record with file and line captured.
#define FAULT_ASSERT(cond) \
    do { if (!(cond)) Diag::FaultAssert(__FILE__, __LINE__); } while(0)
