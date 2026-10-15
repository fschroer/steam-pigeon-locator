# ---------------------------------------------------------------------------
# Launch onset (record t = 0) — host build
#
# Covers Rocket/Common/Inc/LaunchOnset.hpp (header-only): where a record's t = 0
# belongs -- the start of the thrust run that leads into launch detect.
#
# Usage:
#   cd Tests/LaunchOnset
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

TARGET   := test_launch_onset
SRCS     := test_launch_onset.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/LaunchOnset.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
