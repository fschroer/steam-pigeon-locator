# ---------------------------------------------------------------------------
# Full-record codec tests — host build
#
# Covers FullRecordCodec (Rocket/Communication/Inc, header-only): the lossless
# full-sample codec of ADR-0040 (#60).  Bit-exact decode, independent packets,
# every page within the 256-packet ACK bitmap, a deterministic plan, CRC-32.
#   SP_CSV_DIR=<export folder> make run   also measures samples per packet.
#
# Usage:
#   cd Tests/FullRecordCodec
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows use MinGW/MSYS2 g++, or the one
# bundled with STM32CubeIDE.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -I../../Rocket/Communication/Inc \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc \
            -I../../Rocket/Navigation/Inc \
            -I../common

TARGET   := test_full_record_codec
SRCS     := test_full_record_codec.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Communication/Inc/FullRecordCodec.hpp ../../Rocket/Archive/Inc/ArchiveTypes.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
