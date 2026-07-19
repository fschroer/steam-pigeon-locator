# ---------------------------------------------------------------------------
# EKF replay harness — host build (ADR-0004 offline analysis tool)
#
# Compiles the REAL Rocket/Navigation/Src/InsEkf15.cpp and drives it from an
# archived flight CSV.  InsEkf15 pulls in no HAL or driver headers, so it builds
# on host unmodified.
#
# Usage:
#   cd Tests/EkfReplay
#   make                                          # build
#   ./test_ekf_replay flight.csv                  # replay + summary
#   ./test_ekf_replay flight.csv --dump > out.csv # per-sample trace
#   ./test_ekf_replay flight.csv --q-abias 1e-6   # re-tune Launched accel-bias Q
#   ./test_ekf_replay flight.csv --accel-scale 1.05
#
# Requires: g++ (C++17) on PATH.  On Windows use the MinGW/MSYS2 g++ (the ARM
# cross-compiler will not do) and set TMP to a Windows path -- native g++
# rejects an msys-style $TMP and falls back to C:\Windows, which is not
# writable ("Cannot create temporary file").
# ---------------------------------------------------------------------------

CXX      := g++
ROOT     := ../..
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -ffunction-sections -fdata-sections \
            -I$(ROOT)/Rocket/Common/Inc \
            -I$(ROOT)/Rocket/Navigation/Inc \
            -I$(ROOT)/Tests/common
LDFLAGS  := -Wl,--gc-sections

TARGET   := test_ekf_replay
SRCS     := test_ekf_replay.cpp \
            $(ROOT)/Rocket/Navigation/Src/InsEkf15.cpp \
            $(ROOT)/Rocket/Navigation/Src/Math.cpp \
            $(ROOT)/Rocket/Navigation/Src/Cholesky.cpp

.PHONY: all build clean

all: build

build: $(TARGET)

$(TARGET): $(SRCS) $(wildcard $(ROOT)/Rocket/Navigation/Inc/*.hpp) $(ROOT)/Tests/common/FlightCsv.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET) $(TARGET).exe
