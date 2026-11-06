# ---------------------------------------------------------------------------
# FormatTimestamp — host build
#
# Covers Rocket/Common/Src/Format.cpp's FormatTimestamp: every time the locator
# prints carries its zone, local with the app's offset or labeled UTC
# (ADR "Times are stored in UTC and shown with their offset", 0043).
#
# Usage:
#   cd Tests/Timestamp
#   make          # build and run
#   make build    # build only
#   make clean
#
# Requires: g++ (C++17) on PATH.
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g -I../../Rocket/Common/Inc

TARGET   := test_timestamp
SRCS     := test_timestamp.cpp ../../Rocket/Common/Src/Format.cpp

.PHONY: all build run clean

all: build run

build: $(TARGET)

$(TARGET): $(SRCS) ../../Rocket/Common/Inc/Format.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe
