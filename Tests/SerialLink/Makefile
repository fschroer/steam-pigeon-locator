# ---------------------------------------------------------------------------
# SerialLink tests — host build
#
# The USB-console half of the full-record transfer (ADR "The radio download
# becomes the primary data path", decision 6c): the binary-mode frame parser
# and the two-buffer DMA queue, both header-only and HAL-free.
#
# Usage:
#   cd Tests/SerialLink
#   make          # build and run
#   make build    # build only
#   make clean
# ---------------------------------------------------------------------------

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O1 -g \
            -I../TransferPlan/mocks \
            -I../../Rocket/Communication/Inc \
            -I../../Rocket/Archive/Inc \
            -I../../Rocket/Common/Inc \
            -I../../Rocket/Navigation/Inc

TARGET   := test_serial_link
SRCS     := test_serial_link.cpp
HDRS     := ../../Rocket/Communication/Inc/SerialLink.hpp \
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
