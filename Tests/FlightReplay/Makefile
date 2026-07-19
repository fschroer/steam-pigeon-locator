# ---------------------------------------------------------------------------
# Flight-replay harness — host build
#
# Compiles the REAL Rocket/Src/FlightManager.cpp against thin host mocks (mocks/)
# and drives it through a simulated flight to validate the ADR-0003 deployment
# source-selection ladder (issue #10) and the shared apogee detector.
#
# Usage:
#   cd Tests/FlightReplay
#   make            # build and run the self-test suite
#   make build      # build only
#   make clean
#   ./test_flight_replay flight.csv          # replay a recorded flight
#   ./test_flight_replay flight.csv --spike 42350 40      # inject a spike
#   ./test_flight_replay flight.csv --dropout 43000 45000 # inject a dropout
#
# Requires: g++ (C++17) on PATH.  On Windows use the MinGW/MSYS2 g++ or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
ROOT     := ../..
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -ffunction-sections -fdata-sections \
            -Imocks \
            -I$(ROOT)/Rocket/Inc \
            -I$(ROOT)/Rocket/Common/Inc \
            -I$(ROOT)/Rocket/Archive/Inc \
            -I$(ROOT)/Rocket/Navigation/Inc \
            -I$(ROOT)/Tests/common
LDFLAGS  := -Wl,--gc-sections

TARGET   := test_flight_replay
SRCS     := test_flight_replay.cpp \
            mocks/MockEnv.cpp \
            $(ROOT)/Rocket/Src/FlightManager.cpp \
            $(ROOT)/Rocket/Navigation/Src/Math.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(wildcard mocks/*.hpp mocks/*.h)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
