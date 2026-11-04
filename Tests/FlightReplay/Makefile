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
# The replay corpus and these tests' synthetic flights are 20 Hz, so the real
# code is built at the rate the data was recorded at (ADR-0039).  Every
# sample-count constant is SamplesForMs(), so 20 Hz reproduces the old values.
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -DSAMPLES_PER_SECOND=20 \
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
            $(ROOT)/Rocket/Navigation/Src/Math.cpp \
            $(ROOT)/Rocket/Common/Src/PpsDiscipline.c

.PHONY: all build run clean

all: build run

build: $(TARGET)

# The real headers too: a constant changed in FlightManager.hpp (#45's thresholds,
# say) otherwise leaves a stale binary that silently tests the old value.
$(TARGET): $(SRCS) $(wildcard mocks/*.hpp mocks/*.h) $(wildcard $(ROOT)/Rocket/Inc/*.hpp $(ROOT)/Rocket/Common/Inc/*.hpp $(ROOT)/Rocket/Common/Inc/*.h $(ROOT)/Rocket/Archive/Inc/*.hpp)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
