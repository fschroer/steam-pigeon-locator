#include <cstring>
#include <cmath>

#include "FlightProfileCodec.hpp"

namespace FlightProfileCodec {

namespace {

// Saturate to the int16_t delta range.  The former int16_t(std::round(...))
// cast of an out-of-range value is undefined behavior (float -> integer
// overflow); in practice it typically wraps, turning a large positive step
// into a large negative one.  Not observed on hardware — found by reading.
int16_t Sat16(int64_t v) {
    if (v > INT16_MAX) return INT16_MAX;
    if (v < INT16_MIN) return INT16_MIN;
    return static_cast<int16_t>(v);
}

int16_t Sat16(float v) {
    if (!(v == v)) return 0;                       // NaN
    if (v >= static_cast<float>(INT16_MAX)) return INT16_MAX;
    if (v <= static_cast<float>(INT16_MIN)) return INT16_MIN;
    return static_cast<int16_t>(std::lround(v));
}

}  // namespace

// PackSamples encodes sample_count samples into the out_payload buffer.
//
// Wire layout:
//   CompressedHeader          -- absolute values from samples[0]
//   CompressedDelta[0]        -- delta from samples[0] to samples[1]
//   CompressedDelta[1]        -- delta from samples[1] to samples[2]
//   ...
//
// All fields are relative to the previous sample EXCEPT lat/lon, which are
// stored relative to the packet's absolute base (hdr.base_lat_rad /
// hdr.base_lon_rad) to prevent accumulated rounding error.  "Previous sample"
// means the one the decoder reconstructs, and every delta saturates at the
// int16_t range (see the loop).
//
// Returns the number of samples written (not bytes).  May be less than
// sample_count if out_capacity is exhausted.
size_t PackSamples(const FlightArchive::FlightSample* samples,
                   size_t sample_count,
                   uint8_t* out_payload,
                   size_t out_capacity)
{
    if (sample_count == 0) return 0;

    uint8_t*       p   = out_payload;
    const uint8_t* end = out_payload + out_capacity;

    if (p + sizeof(CompressedHeader) > end)
        return 0;

    // Write absolute header from the first sample
    CompressedHeader hdr{};
    hdr.base_timestamp_ms = samples[0].timestamp_ms;
    // Raw baro AGL, not fused: raw baro is the Priority-1 source of record
    // (ADR-0003 / ADR-0005), and the fused columns are the live EKF's purely
    // observational output (ADR-0005 amendment 2026-07-15).  The app's profile
    // chart plots the altitude the deployment decisions were actually made on.
    hdr.base_altitude_m   = samples[0].raw_baro_altitude_agl;
    hdr.base_accel_mps2   = samples[0].accel;
    hdr.base_gyro_dps     = samples[0].gyro;
    hdr.base_lat_rad      = FlightArchive::Deg1e7ToRad(samples[0].lat_1e7);
    hdr.base_lon_rad      = FlightArchive::Deg1e7ToRad(samples[0].lon_1e7);

    std::memcpy(p, &hdr, sizeof(hdr));
    p += sizeof(hdr);

    // `rec` is the sample as the DECODER will reconstruct it, not the true
    // previous sample.  Chaining from the truth let one out-of-range delta (a
    // >3,276.7 dps gyro swing in a step, likelier once the transfer plan
    // decimates the descent) skew every later sample in the packet, and let
    // rounding accumulate along it (ADR-0009 amendment 2026-09-28, invariant
    // 14).  Updated below with the decoder's own float arithmetic, so the two
    // stay bit-identical and the decoders need no change.
    FlightArchive::FlightSample rec = samples[0];
    size_t written = 1;

    for (size_t i = 1; i < sample_count; ++i) {
        if (p + sizeof(CompressedDelta) > end)
            break;

        const auto& s = samples[i];

        CompressedDelta d{};

        // Time and kinematic fields: delta from the reconstructed previous sample
        d.d_timestamp_ms     = Sat16(static_cast<int64_t>(s.timestamp_ms)
                                     - static_cast<int64_t>(rec.timestamp_ms));
        d.d_alt_0p1m         = Sat16((s.raw_baro_altitude_agl - rec.raw_baro_altitude_agl) * 10.0f);
        d.d_accel_x_0p1mps2  = Sat16((s.accel.x    - rec.accel.x)     * 10.0f);
        d.d_accel_y_0p1mps2  = Sat16((s.accel.y    - rec.accel.y)     * 10.0f);
        d.d_accel_z_0p1mps2  = Sat16((s.accel.z    - rec.accel.z)     * 10.0f);
        d.d_gyro_x_0p1dps    = Sat16((s.gyro.x     - rec.gyro.x)      * 10.0f);
        d.d_gyro_y_0p1dps    = Sat16((s.gyro.y     - rec.gyro.y)      * 10.0f);
        d.d_gyro_z_0p1dps    = Sat16((s.gyro.z     - rec.gyro.z)      * 10.0f);

        // Lat/lon: delta from the packet's absolute base, NOT from prev.
        // This keeps accumulated floating-point error out of the decode path.
        d.d_lat_scaled = int32_t(std::round((FlightArchive::Deg1e7ToRad(s.lat_1e7) - hdr.base_lat_rad) * LATLON_SCALE));
        d.d_lon_scaled = int32_t(std::round((FlightArchive::Deg1e7ToRad(s.lon_1e7) - hdr.base_lon_rad) * LATLON_SCALE));

        std::memcpy(p, &d, sizeof(d));
        p += sizeof(d);

        // Advance exactly as UnpackSamples (and the app's decodePayload) will.
        rec.timestamp_ms          = rec.timestamp_ms          + d.d_timestamp_ms;
        rec.raw_baro_altitude_agl = rec.raw_baro_altitude_agl + d.d_alt_0p1m        / 10.0f;
        rec.accel.x               = rec.accel.x               + d.d_accel_x_0p1mps2 / 10.0f;
        rec.accel.y               = rec.accel.y               + d.d_accel_y_0p1mps2 / 10.0f;
        rec.accel.z               = rec.accel.z               + d.d_accel_z_0p1mps2 / 10.0f;
        rec.gyro.x                = rec.gyro.x                + d.d_gyro_x_0p1dps   / 10.0f;
        rec.gyro.y                = rec.gyro.y                + d.d_gyro_y_0p1dps   / 10.0f;
        rec.gyro.z                = rec.gyro.z                + d.d_gyro_z_0p1dps   / 10.0f;
        ++written;
    }

    return written;
}

// UnpackSamples reconstructs samples from a compressed payload buffer.
// Returns the number of samples written into out_samples.
size_t UnpackSamples(const uint8_t* payload,
                     size_t payload_size,
                     FlightArchive::FlightSample* out_samples,
                     size_t max_samples)
{
    if (payload_size < sizeof(CompressedHeader) || max_samples == 0)
        return 0;

    const uint8_t* p   = payload;
    const uint8_t* end = payload + payload_size;

    CompressedHeader hdr{};
    std::memcpy(&hdr, p, sizeof(hdr));
    p += sizeof(hdr);

    // Reconstruct the first sample from the absolute header values
    FlightArchive::FlightSample prev{};
    prev.timestamp_ms = hdr.base_timestamp_ms;
    prev.raw_baro_altitude_agl = hdr.base_altitude_m;
    prev.accel        = hdr.base_accel_mps2;
    prev.gyro         = hdr.base_gyro_dps;
    prev.lat_1e7      = FlightArchive::RadToDeg1e7(hdr.base_lat_rad);
    prev.lon_1e7      = FlightArchive::RadToDeg1e7(hdr.base_lon_rad);

    size_t written = 0;
    out_samples[written++] = prev;

    while (p + sizeof(CompressedDelta) <= end && written < max_samples) {
        CompressedDelta d{};
        std::memcpy(&d, p, sizeof(d));
        p += sizeof(d);

        FlightArchive::FlightSample s{};

        // Kinematic fields: apply delta to previous sample
        s.timestamp_ms = prev.timestamp_ms + d.d_timestamp_ms;
        s.raw_baro_altitude_agl = prev.raw_baro_altitude_agl + d.d_alt_0p1m / 10.0f;
        s.accel.x      = prev.accel.x      + d.d_accel_x_0p1mps2 / 10.0f;
        s.accel.y      = prev.accel.y      + d.d_accel_y_0p1mps2 / 10.0f;
        s.accel.z      = prev.accel.z      + d.d_accel_z_0p1mps2 / 10.0f;
        s.gyro.x       = prev.gyro.x       + d.d_gyro_x_0p1dps   / 10.0f;
        s.gyro.y       = prev.gyro.y       + d.d_gyro_y_0p1dps   / 10.0f;
        s.gyro.z       = prev.gyro.z       + d.d_gyro_z_0p1dps   / 10.0f;

        // Lat/lon: apply delta from the packet's absolute base (matches encode)
        s.lat_1e7 = FlightArchive::RadToDeg1e7(hdr.base_lat_rad + d.d_lat_scaled / LATLON_SCALE);
        s.lon_1e7 = FlightArchive::RadToDeg1e7(hdr.base_lon_rad + d.d_lon_scaled / LATLON_SCALE);

        out_samples[written++] = s;
        prev = s;
    }

    return written;
}

} // namespace FlightProfileCodec
