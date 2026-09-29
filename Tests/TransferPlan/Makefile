# ---------------------------------------------------------------------------
# Flight-data transfer tests — host build
#
# Covers TransferPlan (Rocket/Communication/Inc, header-only) and the real
# FlightProfileCodec.cpp.  Guards issue #49 and the ADR-0009 amendment of
# 2026-09-28: the whole record fits the 256-packet ACK bitmap, full rate at
# events, a deterministic plan, and saturating codec deltas.
#
# Usage:
#   cd Tests/TransferPlan
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -Imocks \
            -I../../Rocket/Communication/Inc \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc \
            -I../../Rocket/Navigation/Inc

TARGET   := test_transfer_plan
SRCS     := test_transfer_plan.cpp \
            ../../Rocket/Communication/Src/FlightProfileCodec.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Communication/Inc/TransferPlan.hpp ../../Rocket/Communication/Inc/FlightProfileCodec.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
