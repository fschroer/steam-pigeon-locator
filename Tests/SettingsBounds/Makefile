# ---------------------------------------------------------------------------
# Deployment settings bounds — host build
#
# Covers Rocket/Archive/Inc/SettingsBounds.hpp (header-only): the bounds table,
# the whole-request check, the boot repair and the push rule.  Guards ADR-0034
# and issue #50.
#
# Usage:
#   cd Tests/SettingsBounds
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc

TARGET   := test_settings_bounds
SRCS     := test_settings_bounds.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Archive/Inc/SettingsBounds.hpp ../../Rocket/Archive/Inc/RocketSettings.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
