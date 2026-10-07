# ---------------------------------------------------------------------------
# Deployment status byte — host build
#
# Covers Rocket/Common/Inc/DeployStats.hpp (header-only): each channel's status
# byte takes only that channel's continuity bit.  Guards issue #52.
#
# Usage:
#   cd Tests/DeployStats
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

TARGET   := test_deploy_stats
SRCS     := test_deploy_stats.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/DeployStats.hpp ../../Rocket/Common/Inc/Constants.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
