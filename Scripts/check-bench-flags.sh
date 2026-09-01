#!/usr/bin/env bash
# check-bench-flags.sh — compile-check the bench harnesses that ship DISABLED.
#
# The locator carries four bench-only harnesses, each behind a compile flag that
# defaults to 0 and must stay 0 in any flight build:
#
#   SP_BENCH_REPLAY  archived-flight replay through the live state machine (#35/#36)
#   SP_FAULT_INJECT  deliberate crashes to validate FaultLog / IWDG (#17)
#   SP_LOSS_INJECT   deterministic RF loss for transfer + channel recovery (#18/#20)
#   SP_VACUUM_SIM    synthetic boost so a vacuum chamber can fly a whole flight
#
# Because they are compiled OUT of every build anyone actually makes, the code
# inside them rots silently: it is never parsed, so a rename in a struct it reads
# breaks it without breaking anything visible.  That is not hypothetical.  The
# flight archive migrated its GPS position fields from `double lat_rad/lon_rad`
# to `int32_t lat_1e7/lon_1e7`, and SP_BENCH_REPLAY kept referencing the old
# names.  The harness stayed broken until someone tried to use it on the bench —
# which is the worst possible moment, because by then it is a debugging tool that
# itself needs debugging.
#
# This script parses each harness at its ENABLED setting so that rot fails here,
# in seconds, instead of on the bench.
#
# It ALSO asserts each flag still defaults to 0, so a flag left enabled after a
# bench session cannot reach a flight build unnoticed.
#
# Usage:
#   Scripts/check-bench-flags.sh              # check every flag
#   Scripts/check-bench-flags.sh SP_BENCH_REPLAY   # check just one
#
# Exit code is 0 only if every flag defaults to 0 AND every enabled
# configuration compiles.
#
# Scope: this is a SYNTAX check (-fsyntax-only), not a link.  It catches the
# entire class of rot described above (renamed/removed/retyped members, changed
# signatures, missing headers).  It does NOT catch a declaration that is never
# defined, which surfaces only at link time — for that, set the flag to 1 in its
# header and run a full `make -j4 all` in Debug/.  Note that
# `make CXXFLAGS+=-DSP_BENCH_REPLAY=1` does NOT work: the CubeIDE-generated
# makefiles hard-code their compile flags, so the flag never reaches the
# compiler and the build silently stays disabled.  Passing -D directly to the
# compiler (as this script does) works, because each flag is declared #ifndef.

set -u

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT" || exit 1

# ── Toolchain ───────────────────────────────────────────────────────────────
# Not on PATH; it ships inside STM32CubeIDE.  $ARM_GXX overrides for other layouts.
if [ -z "${ARM_GXX:-}" ]; then
    ARM_GXX="$(ls -d /c/ST/STM32CubeIDE_*/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*/tools/bin/arm-none-eabi-g++.exe 2>/dev/null | head -1)"
fi
if [ -z "$ARM_GXX" ] || [ ! -x "$ARM_GXX" ]; then
    echo "ERROR: arm-none-eabi-g++ not found." >&2
    echo "       Set ARM_GXX=/path/to/arm-none-eabi-g++ and re-run." >&2
    exit 2
fi

# Communication.cpp includes the generated, gitignored version.h.
[ -f Core/Inc/version.h ] || bash Scripts/GenVersion.sh >/dev/null 2>&1 || true

# ── Build flags: mirrors the Debug configuration ────────────────────────────
CPU="-mcpu=cortex-m4 -mthumb -mfloat-abi=soft --specs=nano.specs"
STD="-std=gnu++17 -fno-exceptions -fno-rtti"
DEFS="-DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL5Mxx"
INC="-ICore/Inc -ISubGHz_Phy/App -ISubGHz_Phy/Target
     -IDrivers/STM32WLxx_HAL_Driver/Inc -IDrivers/STM32WLxx_HAL_Driver/Inc/Legacy
     -IUtilities/trace/adv_trace -IUtilities/misc -IUtilities/sequencer
     -IUtilities/timer -IUtilities/lpm/tiny_lpm
     -IDrivers/CMSIS/Device/ST/STM32WLxx/Include -IDrivers/CMSIS/Include
     -IMiddlewares/Third_Party/SubGHz_Phy/radio_driver
     -IRocket/Inc -IAccelerometer/Inc -IBMP280/Inc
     -IRocket/Archive/Inc -IRocket/Common/Inc -IRocket/Navigation/Inc
     -IRocket/Communication/Inc"

ALL_FLAGS="SP_BENCH_REPLAY SP_FAULT_INJECT SP_LOSS_INJECT SP_VACUUM_SIM"
FLAGS="${*:-$ALL_FLAGS}"

failures=0

for flag in $FLAGS; do
    echo "── $flag ──────────────────────────────────────────────"

    # 1. The committed default must be 0, or a bench build could ship.
    default_line="$(git grep -h -E "^#define +${flag} +[01]" -- '*.hpp' '*.cpp' '*.h' | head -1)"
    if [ -z "$default_line" ]; then
        echo "   FAIL: no '#define $flag <0|1>' found — did the flag get renamed?"
        failures=$((failures + 1))
        continue
    fi
    if echo "$default_line" | grep -qE "^#define +${flag} +0"; then
        echo "   default 0  ok"
    else
        echo "   FAIL: default is not 0 -> '$default_line'"
        echo "         A bench flag left enabled must never reach a flight build."
        failures=$((failures + 1))
    fi

    # 2. Every TU that references the flag must PARSE with it enabled.
    tus="$(git grep -l "$flag" -- '*.cpp')"
    if [ -z "$tus" ]; then
        echo "   FAIL: no .cpp references $flag — harness gone, or flag renamed?"
        failures=$((failures + 1))
        continue
    fi
    for tu in $tus; do
        if out="$("$ARM_GXX" -fsyntax-only $CPU $STD $DEFS "-D${flag}=1" $INC "$tu" 2>&1)"; then
            echo "   compile ${tu}  ok"
        else
            echo "   FAIL: ${tu} does not compile with ${flag}=1"
            echo "$out" | grep -E "error:" | sed 's/^/         /' | head -20
            failures=$((failures + 1))
        fi
    done
done

echo
if [ "$failures" -eq 0 ]; then
    echo "PASS — every bench flag defaults to 0 and compiles when enabled."
    exit 0
fi
echo "FAIL — $failures problem(s) above."
exit 1
