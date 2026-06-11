# ---------------------------------------------------------------------------
# FlightArchive round-trip test — host build
#
# Usage:
#   cd Tests/ArchiveRoundTrip
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.  On Windows with STM32CubeIDE installed,
# use the MinGW shell bundled with the IDE, or any MinGW/MSYS2 g++.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc

TARGET   := test_archive_roundtrip
SRCS     := test_archive_roundtrip.cpp \
            ../../Rocket/Archive/Src/FlightArchiveCrc.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) RamFlashDriver.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
