# ---------------------------------------------------------------------------
# Sensor faults — host build
#
# Covers Rocket/Common/Inc/SensorFaults.hpp (header-only): which health values
# count as a failed sensor, and which failures refuse arming.  Guards ADR-0038.
#
# Usage:
#   cd Tests/SensorFaults
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -I../../Rocket/Common/Inc

TARGET   := test_sensor_faults
SRCS     := test_sensor_faults.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/SensorFaults.hpp ../../Rocket/Common/Inc/Types.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
