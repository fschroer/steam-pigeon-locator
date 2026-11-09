# ---------------------------------------------------------------------------
# Horizontal bridge (#72) — host build
#
# Covers Rocket/Navigation/Inc/HorizontalBridge.hpp (header-only), the
# horizontal half of the locator's fused navigation: the GPS fix while it is
# trusted, bridged across an outage by flight phase otherwise.  `make` runs the
# synthetic unit tests (what the pre-commit hook runs).  `make replay` injects
# GPS outages into the flight corpus and scores the bridged position against
# the trusted fix at each outage's end, with pass/fail thresholds.  The corpus
# is not in git, so point FLIGHT_DATA at it:
#
#   cd Tests/HorizontalBridge
#   make
#   make replay FLIGHT_DATA="G:/My Drive/Rocketry/Flight Data"
#
# Requires: g++ (C++17) on PATH.
# ---------------------------------------------------------------------------

CXX      := g++
ROOT     := ../..
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -DSAMPLES_PER_SECOND=20 \
            -I$(ROOT)/Rocket/Common/Inc \
            -I$(ROOT)/Rocket/Navigation/Inc \
            -I$(ROOT)/Tests/common

TARGET   := test_horizontal_bridge
SRCS     := test_horizontal_bridge.cpp $(ROOT)/Rocket/Navigation/Src/Math.cpp
FLIGHT_DATA ?= G:/My Drive/Rocketry/Flight Data

.PHONY: all build run replay clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(ROOT)/Rocket/Navigation/Inc/HorizontalBridge.hpp $(ROOT)/Rocket/Navigation/Inc/GpsVerticalCrossCheck.hpp $(ROOT)/Tests/common/FlightCsv.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

replay: $(TARGET)
	./$(TARGET) --corpus "$(FLIGHT_DATA)"

clean:
	rm -f $(TARGET) $(TARGET).exe
