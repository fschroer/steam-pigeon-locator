# ---------------------------------------------------------------------------
# Vertical filter (#71) — host build
#
# Covers Rocket/Navigation/Inc/VerticalFilter.hpp (header-only), the 2-state
# altitude / vertical-speed filter proposed to take the EKF's observational
# role.  `make` runs the synthetic unit tests (what the pre-commit hook runs).
# `make replay` scores it on the flight corpus against GPS vertical speed with
# pass/fail thresholds; the corpus is not in git, so point FLIGHT_DATA at it:
#
#   cd Tests/VerticalFilter
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

TARGET   := test_vertical_filter
SRCS     := test_vertical_filter.cpp $(ROOT)/Rocket/Navigation/Src/Math.cpp
FLIGHT_DATA ?= G:/My Drive/Rocketry/Flight Data

.PHONY: all build run replay clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(ROOT)/Rocket/Navigation/Inc/VerticalFilter.hpp $(ROOT)/Tests/common/FlightCsv.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

replay: $(TARGET)
	./$(TARGET) --corpus "$(FLIGHT_DATA)"

clean:
	rm -f $(TARGET) $(TARGET).exe
