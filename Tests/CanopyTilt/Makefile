# ---------------------------------------------------------------------------
# Canopy tilt correction (#44) and GPS vertical cross-check (#70) — host build
#
# Compiles the REAL AttitudeEstimator.cpp + Math.cpp and the header-only
# CanopyTiltGate / GpsVerticalCrossCheck.  `make` runs the unit tests; pass a
# flight CSV to replay its descent through the real estimator:
#
#   cd Tests/CanopyTilt
#   make                      # build and run the unit tests
#   ./test_canopy_tilt <flight.csv>   # replay one record (20 Hz corpus)
#   make clean
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

TARGET   := test_canopy_tilt
SRCS     := test_canopy_tilt.cpp \
            $(ROOT)/Rocket/Navigation/Src/AttitudeEstimator.cpp \
            $(ROOT)/Rocket/Navigation/Src/Math.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(ROOT)/Rocket/Navigation/Inc/CanopyTiltGate.hpp \
           $(ROOT)/Rocket/Navigation/Inc/GpsVerticalCrossCheck.hpp \
           $(ROOT)/Rocket/Navigation/Inc/AttitudeEstimator.hpp \
           $(ROOT)/Tests/common/FlightCsv.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
