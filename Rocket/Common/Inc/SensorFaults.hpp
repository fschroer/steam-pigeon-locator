#pragma once
// ---------------------------------------------------------------------------
// SensorFaults — what a failed sensor takes away, and what it does not.
//
// ADR-0038: a sensor that fails its power-on check is REPORTED, not fatal.  The
// locator used to restart after 3 s of red LED whenever any one of the IMU,
// barometer or GPS failed init, and kept restarting for as long as the fault
// stood.  After a ballistic flight that damaged one sensor, that loop locked the
// operator out of the one thing still worth having — the stored flight record —
// because neither the USB-C console nor the app's download could be reached in
// the few seconds between restarts.
//
// The rule, kept here in one place so the firmware gate and its host test cannot
// disagree (the Android app mirrors it in SensorFaults.kt — change both):
//
//   * A sensor has FAILED when its health is Error, or Off / Initializing after
//     Navigation::Init() — i.e. it is not running.  Warning (a transient read
//     failure) and Stale (a GPS with no fresh fix) are NOT failures: the sensor
//     is up and may recover on the next read.
//   * Arming needs the IMU AND the barometer.  Launch and burnout are read from
//     the accelerometer; apogee, main and landing from raw baro (ADR-0003).  A
//     locator missing either would fire charges on a guess, so it refuses
//     ArmRequest.  Arming gates pyro only (ADR-0021), so refusing it costs
//     deployments and nothing else.
//   * The GPS gates nothing.  Without it the locator still detects, deploys and
//     records; it just cannot say where it is.
//
// Header-only and free of HAL so Tests/SensorFaults can compile it on the host.
// ---------------------------------------------------------------------------
#include <cstdint>

#include "Types.hpp"

namespace SensorFaults {

constexpr bool Failed(SensorHealth h) {
    return h == SensorHealth::Error
        || h == SensorHealth::Off
        || h == SensorHealth::Initializing;
}

constexpr bool ArmingAllowed(SensorHealth imu, SensorHealth baro) {
    return !Failed(imu) && !Failed(baro);
}

// One bit per failed sensor, for "is anything wrong?" tests and the console.
constexpr uint8_t kImu  = 0x01;
constexpr uint8_t kBaro = 0x02;
constexpr uint8_t kGps  = 0x04;

constexpr uint8_t FailedMask(SensorHealth imu, SensorHealth baro, SensorHealth gps) {
    return static_cast<uint8_t>((Failed(imu)  ? kImu  : 0)
                              | (Failed(baro) ? kBaro : 0)
                              | (Failed(gps)  ? kGps  : 0));
}

} // namespace SensorFaults
