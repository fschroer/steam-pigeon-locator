# ---------------------------------------------------------------------------
# ParityRing tests — host build
#
# The quick-look transfer's parity accumulators, cut from one per possible
# group (64) to a ring of 4.  The test flies a lossy windowed transfer through
# the old array and the ring side by side and requires the same parity packets.
#
# Usage:
#   cd Tests/ParityRing
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

TARGET   := test_parity_ring
SRCS     := test_parity_ring.cpp
HDRS     := ../../Rocket/Communication/Inc/ParityRing.hpp \
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
