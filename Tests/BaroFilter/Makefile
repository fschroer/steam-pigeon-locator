# ---------------------------------------------------------------------------
# Baro conditioning tests — host build
#
# Covers MedianFilter<N> (Rocket/Common/Inc) and VelocityEstimator<N>
# (Rocket/Navigation/Inc/Velocity.tpp).  Both are HAL-free headers, so this
# needs no mocks at all — it compiles the real code the firmware runs.
#
# Guards ADR-0032 (median-5 on pressure ahead of the IIR) and issue #41
# (the retired ±200 m/s step clamp).
#
# Usage:
#   cd Tests/BaroFilter
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -I../../Rocket/Common/Inc \
            -I../../Rocket/Navigation/Inc

TARGET   := test_baro_filter
SRCS     := test_baro_filter.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/MedianFilter.hpp ../../Rocket/Navigation/Inc/Velocity.tpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
