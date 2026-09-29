#pragma once
// Host stub for Rocket/Common/Inc/DeviceUID.hpp, whose constructor reads the
// STM32 unique-ID registers through the HAL.
//
// The codec reaches it only by include chain (FlightProfileCodec.hpp ->
// MessageProtocol.hpp -> Archive.hpp -> DeviceUID.hpp) and never constructs
// one.  Stubbing this one header lets the suite compile the REAL Archive.hpp,
// so record_count and the MessageProtocol static_asserts are checked against
// the firmware's own values rather than a copy that could drift.
#include <cstdint>

class DeviceUID {
public:
    uint32_t getUID() const { return 0u; }
};
