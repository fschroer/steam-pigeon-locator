# ---------------------------------------------------------------------------
# Variable-rate archive policy (ADR-0039) -- host build.
#   make        build and run
#   make clean
# ---------------------------------------------------------------------------
CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O0 -g -I../../Rocket/Common/Inc
TARGET   := test_archive_rate_policy
SRCS     := test_archive_rate_policy.cpp

.PHONY: all build run clean
all: build run
build: $(TARGET)
$(TARGET): $(SRCS) ../../Rocket/Common/Inc/ArchiveRatePolicy.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
run: $(TARGET)
	./$(TARGET)
clean:
	rm -f $(TARGET) $(TARGET).exe
