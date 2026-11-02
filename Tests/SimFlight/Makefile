# ---------------------------------------------------------------------------
# SimFlight tests — host build
#
# The app-started simulated flight (ADR "An app-started simulated flight
# computed on the locator", 0041).  Flies SimFlight::Generator's profiles
# through the REAL Rocket/Src/FlightManager.cpp, using FlightReplay's mocks for
# Navigation / Archive / Deployment / HAL, with the canopies opened by the state
# machine's own deployment events, and checks each event against the formula.
#
# Built at the firmware's own loop rate (50 Hz), unlike FlightReplay, whose
# corpus is 20 Hz.
#
# Usage:
#   cd Tests/SimFlight
#   make          # build and run
#   make build    # build only
#   make clean
# ---------------------------------------------------------------------------

CXX      := g++
ROOT     := ../..
MOCKS    := ../FlightReplay/mocks
CXXFLAGS := -std=c++17 -Wall -Wextra -O1 -g \
            -ffunction-sections -fdata-sections \
            -I$(MOCKS) \
            -I$(ROOT)/Rocket/Inc \
            -I$(ROOT)/Rocket/Common/Inc \
            -I$(ROOT)/Rocket/Archive/Inc \
            -I$(ROOT)/Rocket/Navigation/Inc \
            -I$(ROOT)/Tests/common
LDFLAGS  := -Wl,--gc-sections

TARGET   := test_sim_flight
SRCS     := test_sim_flight.cpp \
            $(MOCKS)/MockEnv.cpp \
            $(ROOT)/Rocket/Src/FlightManager.cpp \
            $(ROOT)/Rocket/Navigation/Src/Math.cpp \
            $(ROOT)/Rocket/Common/Src/PpsDiscipline.c

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(wildcard $(MOCKS)/*.hpp $(MOCKS)/*.h) $(ROOT)/Rocket/Navigation/Inc/SimFlight.hpp $(wildcard $(ROOT)/Rocket/Inc/*.hpp $(ROOT)/Rocket/Common/Inc/*.hpp $(ROOT)/Rocket/Archive/Inc/*.hpp)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
