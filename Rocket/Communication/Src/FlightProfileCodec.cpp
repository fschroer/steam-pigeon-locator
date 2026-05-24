#include <cstring>
#include <cmath>

#include "FlightProfileCodec.hpp"

namespace FlightProfileCodec {

inline size_t PackSamples(const FlightArchive::FlightSample* samples,
                          size_t max_samples,
                          uint8_t* out_payload,
                          size_t out_capacity)
{
    if (max_samples == 0) return 0;

    uint8_t* p   = out_payload;
    auto*    end = out_payload + out_capacity;

    if (p + sizeof(CompressedHeader) > end)
        return 0;

    CompressedHeader hdr{};
    hdr.base_timestamp_ms = samples[0].timestamp_ms;
    hdr.base_altitude_m   = samples[0].altitude_m;
    hdr.base_accel_mps2   = samples[0].accel;
    hdr.base_gyro_dps     = samples[0].gyro;
    hdr.base_lat_rad      = samples[0].lat_rad;
    hdr.base_lon_rad      = samples[0].lon_rad;

    std::memcpy(p, &hdr, sizeof(hdr));
    p += sizeof(hdr);

    FlightArchive::FlightSample prev = samples[0];
    size_t written = 1;

    for (size_t i = 1; i < max_samples; ++i) {
        if (p + sizeof(CompressedDelta) > end)
            break;

        const auto& s = samples[i];

        CompressedDelta d{};
        d.d_timestamp_ms = int16_t(s.timestamp_ms - prev.timestamp_ms);
        d.d_alt_0p1m     = int16_t(std::round((s.altitude_m - prev.altitude_m) * 10.0f));

        d.d_accel_x_0p1mps2 = int16_t(std::round((s.accel.x - prev.accel.x) * 10.0f));
        d.d_accel_y_0p1mps2 = int16_t(std::round((s.accel.y - prev.accel.y) * 10.0f));
        d.d_accel_z_0p1mps2 = int16_t(std::round((s.accel.z - prev.accel.z) * 10.0f));

        d.d_gyro_x_0p1dps = int16_t(std::round((s.gyro.x - prev.gyro.x) * 10.0f));
        d.d_gyro_y_0p1dps = int16_t(std::round((s.gyro.y - prev.gyro.y) * 10.0f));
        d.d_gyro_z_0p1dps = int16_t(std::round((s.gyro.z - prev.gyro.z) * 10.0f));

        d.d_lat_scaled = int32_t(std::round((s.lat_rad - hdr.base_lat_rad) * LATLON_SCALE));
        d.d_lon_scaled = int32_t(std::round((s.lon_rad - hdr.base_lon_rad) * LATLON_SCALE));

        std::memcpy(p, &d, sizeof(d));
        p += sizeof(d);

        prev = s;
        ++written;
    }

    return written;
}

inline size_t UnpackSamples(const uint8_t* payload,
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

    size_t written = 0;

    FlightArchive::FlightSample prev{};
    prev.timestamp_ms = hdr.base_timestamp_ms;
    prev.altitude_m   = hdr.base_altitude_m;
    prev.accel        = hdr.base_accel_mps2;
    prev.gyro         = hdr.base_gyro_dps;
    prev.lat_rad      = hdr.base_lat_rad;
    prev.lon_rad      = hdr.base_lon_rad;

    out_samples[written++] = prev;

    while (p + sizeof(CompressedDelta) <= end && written < max_samples) {
        CompressedDelta d{};
        std::memcpy(&d, p, sizeof(d));
        p += sizeof(d);

        FlightArchive::FlightSample s{};

        s.timestamp_ms = prev.timestamp_ms + d.d_timestamp_ms;
        s.altitude_m   = prev.altitude_m + d.d_alt_0p1m / 10.0f;

        s.accel.x = prev.accel.x + d.d_accel_x_0p1mps2 / 10.0f;
        s.accel.y = prev.accel.y + d.d_accel_y_0p1mps2 / 10.0f;
        s.accel.z = prev.accel.z + d.d_accel_z_0p1mps2 / 10.0f;

        s.gyro.x = prev.gyro.x + d.d_gyro_x_0p1dps / 10.0f;
        s.gyro.y = prev.gyro.y + d.d_gyro_y_0p1dps / 10.0f;
        s.gyro.z = prev.gyro.z + d.d_gyro_z_0p1dps / 10.0f;

        s.lat_rad = hdr.base_lat_rad + d.d_lat_scaled / LATLON_SCALE;
        s.lon_rad = hdr.base_lon_rad + d.d_lon_scaled / LATLON_SCALE;

        out_samples[written++] = s;
        prev = s;
    }

    return written;
}

} // namespace FlightProfileCodec
