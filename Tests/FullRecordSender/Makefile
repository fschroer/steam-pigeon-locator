# ---------------------------------------------------------------------------
# Background full-record transfer tests — host build
#
# Runs the real FullRecordSender (Rocket/Communication/Inc, header-only)
# against a simulated half-duplex LoRa link and app, and the PickupDetector
# that is one of its start triggers.  ADR "The radio download becomes the
# primary data path" (#60).
#
# Usage:
#   cd Tests/FullRecordSender
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O1 -g \
            -I../TransferPlan/mocks \
            -I../../Rocket/Communication/Inc \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc \
            -I../../Rocket/Navigation/Inc

TARGET   := test_full_record_sender
SRCS     := test_full_record_sender.cpp
HDRS     := ../../Rocket/Communication/Inc/FullRecordSender.hpp \
            ../../Rocket/Communication/Inc/FullRecordCodec.hpp \
            ../../Rocket/Communication/Inc/PickupDetector.hpp \
            ../../Rocket/Communication/Inc/MessageProtocol.hpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) $(HDRS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
