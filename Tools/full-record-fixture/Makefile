# Builds the fixture the Android app's FullRecordCodecTest reads.
#   make            # writes $(OUT)
#   make OUT=path   # somewhere else
# Default OUT assumes the four repos are siblings (see the iOS repo's CLAUDE.md).
L   := ../..
OUT ?= ../../../rocket-flight-manager/app/src/test/resources/fullrecord/sender_fixture.bin
CXXFLAGS := -std=c++17 -O1 -I$(L)/Tests/TransferPlan/mocks -I$(L)/Rocket/Communication/Inc \
            -I$(L)/Rocket/Archive/Inc -I$(L)/Rocket/Common/Inc -I$(L)/Rocket/Navigation/Inc

all: gen
	./gen $(OUT)

gen: gen.cpp $(L)/Rocket/Common/Src/Format.cpp
	g++ $(CXXFLAGS) gen.cpp $(L)/Rocket/Common/Src/Format.cpp -o gen

clean:
	rm -f gen
.PHONY: all clean
