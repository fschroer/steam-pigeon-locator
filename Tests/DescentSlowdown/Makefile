# ---------------------------------------------------------------------------
# Descent slowdown (physical deployment inference) — host build
#
# Covers Rocket/Common/Inc/DescentSlowdown.hpp (header-only): the velocity-only
# drogue / main opening inference.  Guards issue #54.
#
# Usage:
#   cd Tests/DescentSlowdown
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

TARGET   := test_descent_slowdown
SRCS     := test_descent_slowdown.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/DescentSlowdown.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
