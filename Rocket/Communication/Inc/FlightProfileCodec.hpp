#pragma once

#include <cstddef>
#include <cstdint>

#include "Types.hpp"
#include "ArchiveTypes.hpp"
#include "MessageProtocol.hpp"

namespace FlightProfileCodec {

constexpr double LATLON_SCALE = 1e7; // ~1 cm precision

struct CompressedHeader {
    uint32_t base_timestamp_ms;
    float    base_altitude_m;
    Vec3f    base_accel_mps2;
    Vec3f    base_gyro_dps;
    double   base_lat_rad;
    double   base_lon_rad;
};

struct CompressedDelta {
    int16_t d_timestamp_ms;
    int16_t d_alt_0p1m;

    int16_t d_accel_x_0p1mps2;
    int16_t d_accel_y_0p1mps2;
    int16_t d_accel_z_0p1mps2;

    int16_t d_gyro_x_0p1dps;
    int16_t d_gyro_y_0p1dps;
    int16_t d_gyro_z_0p1dps;

    // Lat/lon deltas are stored relative to the packet's absolute base
    // (CompressedHeader), not relative to the previous sample, to prevent
    // accumulated rounding error across a long flight.
    int32_t d_lat_scaled;   // (lat_rad - base_lat_rad) * LATLON_SCALE
    int32_t d_lon_scaled;   // (lon_rad - base_lon_rad) * LATLON_SCALE
};

// Encode up to sample_count samples into out_payload.
// Returns the number of samples actually written (may be less than
// sample_count if out_capacity is exhausted).
// NOTE: returns a sample count, not a byte count.
size_t PackSamples(const FlightArchive::FlightSample* samples,
                   size_t sample_count,
                   uint8_t* out_payload,
                   size_t out_capacity);

// Decode samples from a compressed payload buffer.
// Returns the number of samples written into out_samples.
size_t UnpackSamples(const uint8_t* payload,
                     size_t payload_size,
                     FlightArchive::FlightSample* out_samples,
                     size_t max_samples);

// How many samples fit in one packet payload.
// Used as a constexpr so callers can size static buffers against it.
inline constexpr size_t MaxSamplesPerPacket() {
    return 1 + (Communication::kPayloadSize - sizeof(CompressedHeader))
                 / sizeof(CompressedDelta);
}

} // namespace FlightProfileCodec
