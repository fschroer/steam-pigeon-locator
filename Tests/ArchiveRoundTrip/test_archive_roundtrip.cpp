// ---------------------------------------------------------------------------
// test_archive_roundtrip.cpp
//
// Host-side round-trip test for the FlightArchive storage engine.
//
// Tests verified:
//   1.  Single record write and close succeeds
//   2.  ScanArchive after a simulated power cycle finds the record
//   3.  GetFlightSampleCount returns the exact number of samples written
//   4.  ReadFlightDataRange recovers every sample in correct order
//   5.  Every FlightSample field round-trips exactly:
//         timestamp_ms, raw_baro_altitude_agl, fused_altitude_agl,
//         raw_baro_velocity, fused_vertical_speed_mps,
//         accel (body-frame Vec3f), gyro, lat_rad, lon_rad,
//         flight_state  <-- newly added field
//   6.  All Statistic event stats round-trip (LaunchTimestampMs,
//       BurnoutTimestampMs, NoseoverTimestampMs, MaxAltitudeM, ...)
//   7.  A second record can be opened and written independently
//   8.  A truncated (not-closed) record can be recovered via
//       RecoverFlightData and returns a non-zero sample count
// ---------------------------------------------------------------------------

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cassert>
#include <vector>
#include <string>

#include "RamFlashDriver.hpp"
#include "FlightArchive.hpp"
#include "ArchiveTypes.hpp"
#include "SystemFlashLayout.hpp"

// ---------------------------------------------------------------------------
// Test harness helpers
// ---------------------------------------------------------------------------

static int  g_pass = 0;
static int  g_fail = 0;

#define CHECK(expr)                                                            \
    do {                                                                       \
        if (expr) {                                                            \
            printf("  PASS  %s\n", #expr);                                    \
            ++g_pass;                                                          \
        } else {                                                               \
            printf("  FAIL  %s  (line %d)\n", #expr, __LINE__);               \
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

#define CHECK_EQ(a, b)                                                         \
    do {                                                                       \
        auto _a = (a); auto _b = (b);                                          \
        if (_a == _b) {                                                        \
            ++g_pass;                                                          \
        } else {                                                               \
            printf("  FAIL  %s == %s  (%s:%d)\n", #a, #b, __FILE__, __LINE__);\
            ++g_fail;                                                          \
        }                                                                      \
    } while (0)

static bool floatEq(float a, float b)  { return std::memcmp(&a, &b, 4) == 0; }
static bool doubleEq(double a, double b) { return std::memcmp(&a, &b, 8) == 0; }
static bool vec3Eq(const Vec3f& a, const Vec3f& b)
{
    return floatEq(a.x, b.x) && floatEq(a.y, b.y) && floatEq(a.z, b.z);
}

// ---------------------------------------------------------------------------
// Flash / archive configuration — mirrors Archive.cpp exactly
// ---------------------------------------------------------------------------

static constexpr uint32_t kFlashSize           = 8u * 1024u * 1024u;
static constexpr uint32_t kPersistentRegionSize = 32u * 1024u;
static constexpr uint32_t kRuntimeRegionSize    = 32u * 1024u;
static constexpr uint16_t kRecordCount          = 10u;
static constexpr uint16_t kMinutesPerRecord     = 8u;
static constexpr uint16_t kStatSlotCount =
    static_cast<uint16_t>(FlightArchive::Statistic::Count);
static constexpr size_t   kChunkPayloadBytes    = 512u;

using RocketArchive = FlightArchive::Archive<
    FlightArchive::FlightSample,
    FlightArchive::ExampleEventStats,
    kChunkPayloadBytes>;

static RocketArchive::Config MakeConfig()
{
    constexpr SystemFlashLayout layout =
        MakeSystemFlashLayout(kFlashSize, kPersistentRegionSize, kRuntimeRegionSize);
    RocketArchive::Config cfg{};
    cfg.archiveBaseAddress = layout.archiveBaseAddress;
    cfg.archiveSizeBytes   = layout.archiveSizeBytes;
    cfg.recordCount        = kRecordCount;
    cfg.minutesPerRecord   = kMinutesPerRecord;
    cfg.statSlotCount      = kStatSlotCount;
    return cfg;
}

// ---------------------------------------------------------------------------
// Synthetic flight data generator
//
// Returns a deterministic FlightSample for sample index i so that every field
// is independently verifiable and exercises all non-default values including
// the new body-accel and flight_state fields.
// ---------------------------------------------------------------------------

static FlightArchive::FlightSample MakeSample(uint32_t i)
{
    FlightArchive::FlightSample s{};
    s.timestamp_ms            = 1000u + i * 50u;          // 20 Hz starting at T+1 s
    s.raw_baro_altitude_agl   = 10.0f + static_cast<float>(i) * 0.5f;
    s.fused_altitude_agl      = 10.1f + static_cast<float>(i) * 0.5f;
    s.raw_baro_velocity       = 1.0f  + static_cast<float>(i) * 0.1f;
    s.fused_vertical_speed_mps= 1.05f + static_cast<float>(i) * 0.1f;

    // Body-frame accelerometer: distinctive per-sample values
    // (gravity-inclusive, so at rest this would be ~9.81 on one axis;
    //  during burn it climbs — we just use synthetic data here)
    s.accel = { 0.1f * static_cast<float>(i),
                0.2f * static_cast<float>(i),
                9.81f + 0.3f * static_cast<float>(i) };

    s.gyro  = { 0.01f * static_cast<float>(i),
               -0.01f * static_cast<float>(i),
                0.005f * static_cast<float>(i) };

    s.lat_rad = 0.6981317 + static_cast<double>(i) * 1e-7;  // ~40° N
    s.lon_rad = -1.2566371 + static_cast<double>(i) * 1e-7; // ~-72° (Connecticut)

    // Flight state follows a realistic progression:
    //   samples   0– 99 : Launched
    //   samples 100–149 : Burnout
    //   samples 150–199 : Noseover
    //   samples 200+    : Landed
    if      (i < 100)  s.flight_state = static_cast<uint8_t>(FlightStates::Launched);
    else if (i < 150)  s.flight_state = static_cast<uint8_t>(FlightStates::Burnout);
    else if (i < 200)  s.flight_state = static_cast<uint8_t>(FlightStates::Noseover);
    else               s.flight_state = static_cast<uint8_t>(FlightStates::Landed);

    // NFR-9 strapdown attitude (packed int16) — distinctive, in-range per sample.
    s.tilt_cdeg   = static_cast<int16_t>(i % 18001u);              // 0..180.00°
    s.quat_q15[0] = static_cast<int16_t>(32767 - static_cast<int>(i % 1000u));
    s.quat_q15[1] = static_cast<int16_t>(static_cast<int>(i % 655u) * 50);
    s.quat_q15[2] = static_cast<int16_t>(-static_cast<int>(i % 655u) * 50);
    s.quat_q15[3] = static_cast<int16_t>(i % 32768u);

    return s;
}

static void CheckSampleMatch(const FlightArchive::FlightSample& got,
                             const FlightArchive::FlightSample& expected,
                             uint32_t index)
{
    bool ok = (got.timestamp_ms             == expected.timestamp_ms)
           && floatEq(got.raw_baro_altitude_agl,  expected.raw_baro_altitude_agl)
           && floatEq(got.fused_altitude_agl,      expected.fused_altitude_agl)
           && floatEq(got.raw_baro_velocity,        expected.raw_baro_velocity)
           && floatEq(got.fused_vertical_speed_mps, expected.fused_vertical_speed_mps)
           && vec3Eq(got.accel,                     expected.accel)
           && vec3Eq(got.gyro,                      expected.gyro)
           && doubleEq(got.lat_rad,                 expected.lat_rad)
           && doubleEq(got.lon_rad,                 expected.lon_rad)
           && (got.flight_state                == expected.flight_state)
           && (got.tilt_cdeg                   == expected.tilt_cdeg)
           && (got.quat_q15[0] == expected.quat_q15[0])
           && (got.quat_q15[1] == expected.quat_q15[1])
           && (got.quat_q15[2] == expected.quat_q15[2])
           && (got.quat_q15[3] == expected.quat_q15[3]);

    if (!ok) {
        printf("  FAIL  sample[%u] field mismatch\n", index);

        if (got.timestamp_ms != expected.timestamp_ms)
            printf("        timestamp_ms: got %u, expected %u\n",
                   got.timestamp_ms, expected.timestamp_ms);
        if (!floatEq(got.raw_baro_altitude_agl, expected.raw_baro_altitude_agl))
            printf("        raw_baro_altitude_agl: got %.3f, expected %.3f\n",
                   (double)got.raw_baro_altitude_agl, (double)expected.raw_baro_altitude_agl);
        if (!vec3Eq(got.accel, expected.accel))
            printf("        accel: got (%.3f,%.3f,%.3f), expected (%.3f,%.3f,%.3f)\n",
                   (double)got.accel.x, (double)got.accel.y, (double)got.accel.z,
                   (double)expected.accel.x, (double)expected.accel.y, (double)expected.accel.z);
        if (got.flight_state != expected.flight_state)
            printf("        flight_state: got %u, expected %u\n",
                   got.flight_state, expected.flight_state);
        ++g_fail;
    } else {
        ++g_pass;
    }
}

// ---------------------------------------------------------------------------
// Test 1: write, close, scan, read back
// ---------------------------------------------------------------------------

static void TestSingleRecordRoundTrip(RamFlashDriver& flash)
{
    printf("\n--- Test 1: single record round-trip ---\n");

    RocketArchive archive(flash, MakeConfig());
    CHECK(archive.Init());
    CHECK(archive.ScanArchive());

    // Open record 0
    const uint16_t recId = archive.GetNextAvailableArchiveRecord();
    CHECK_EQ(recId, 0u);
    CHECK(archive.PrepareRecord(recId));
    CHECK(archive.InitializeFlightRecord(recId));

    // Write stat events
    const uint32_t kLaunchMs   = 1000u;
    const uint32_t kBurnoutMs  = 6000u;
    const uint32_t kNoseoverMs = 14000u;
    const float    kMaxAlt     = 312.5f;

    CHECK(archive.WriteStat(recId, FlightArchive::Statistic::LaunchTimestampMs,   kLaunchMs));
    CHECK(archive.WriteStat(recId, FlightArchive::Statistic::BurnoutTimestampMs,  kBurnoutMs));
    CHECK(archive.WriteStat(recId, FlightArchive::Statistic::NoseoverTimestampMs, kNoseoverMs));
    CHECK(archive.WriteStat(recId, FlightArchive::Statistic::MaxAltitudeM,        kMaxAlt));

    // Write 250 samples (spans multiple 512-byte chunks;
    // one chunk holds 512 / sizeof(FlightSample) samples)
    const uint32_t kSampleCount = 250u;
    printf("  FlightSample size: %zu bytes\n", sizeof(FlightArchive::FlightSample));
    printf("  Samples per chunk: %zu\n",
           kChunkPayloadBytes / sizeof(FlightArchive::FlightSample));

    for (uint32_t i = 0; i < kSampleCount; ++i)
    {
        FlightArchive::FlightSample s = MakeSample(i);
        CHECK(archive.WriteFlightDataSample(recId, s));
    }

    // Flush and close
    CHECK(archive.FlushFlightData(recId));
    CHECK(archive.CloseFlightRecord(recId));
    CHECK(archive.SetRecordValid(recId));

    // -----------------------------------------------------------------------
    // Simulate a power cycle: construct a new Archive object over the same RAM
    // -----------------------------------------------------------------------
    printf("  [simulating power cycle — rescanning]\n");
    RocketArchive archive2(flash, MakeConfig());
    CHECK(archive2.Init());
    CHECK(archive2.ScanArchive());

    // Sample count
    uint32_t countOut = 0;
    CHECK(archive2.GetFlightSampleCount(recId, countOut));
    CHECK_EQ(countOut, kSampleCount);

    // Read back in chunks of 64 and verify every sample
    {
        FlightArchive::FlightSample buf[64];
        uint32_t start = 0u;
        uint32_t totalRead = 0u;
        bool allMatch = true;

        while (true)
        {
            uint32_t got = 0u;
            if (!archive2.ReadFlightDataRange(recId, start, buf, 64u, got)) break;
            if (got == 0u) break;

            for (uint32_t i = 0u; i < got; ++i)
            {
                FlightArchive::FlightSample expected = MakeSample(start + i);
                CheckSampleMatch(buf[i], expected, start + i);
            }
            start      += got;
            totalRead  += got;
        }

        CHECK_EQ(totalRead, kSampleCount);
        (void)allMatch;
    }

    // Read back stat events
    {
        uint32_t v = 0u;
        float    fv = 0.0f;
        bool     present = false;

        CHECK(archive2.ReadStat(recId, FlightArchive::Statistic::LaunchTimestampMs,   v,  present));
        CHECK(present);
        CHECK_EQ(v, kLaunchMs);

        CHECK(archive2.ReadStat(recId, FlightArchive::Statistic::BurnoutTimestampMs,  v,  present));
        CHECK(present);
        CHECK_EQ(v, kBurnoutMs);

        CHECK(archive2.ReadStat(recId, FlightArchive::Statistic::NoseoverTimestampMs, v,  present));
        CHECK(present);
        CHECK_EQ(v, kNoseoverMs);

        CHECK(archive2.ReadStat(recId, FlightArchive::Statistic::MaxAltitudeM,        fv, present));
        CHECK(present);
        CHECK(floatEq(fv, kMaxAlt));
    }

    // Absent stat should come back not-present
    {
        uint32_t v = 0u;
        bool present = false;
        CHECK(archive2.ReadStat(recId, FlightArchive::Statistic::DroguePrimaryDeployTimestampMs, v, present));
        CHECK(!present);
    }
}

// ---------------------------------------------------------------------------
// Test 2: second record is independent of the first
// ---------------------------------------------------------------------------

static void TestSecondRecordIndependent(RamFlashDriver& flash)
{
    printf("\n--- Test 2: second record independent ---\n");

    RocketArchive archive(flash, MakeConfig());
    CHECK(archive.Init());
    CHECK(archive.ScanArchive());

    const uint16_t recId = archive.GetNextAvailableArchiveRecord();
    CHECK_EQ(recId, 1u);  // slot 0 already used
    CHECK(archive.PrepareRecord(recId));
    CHECK(archive.InitializeFlightRecord(recId));

    const uint32_t kSampleCount = 40u;  // small flight
    for (uint32_t i = 0; i < kSampleCount; ++i)
    {
        // Use a different seed so values differ from record 0
        FlightArchive::FlightSample s = MakeSample(i + 1000u);
        CHECK(archive.WriteFlightDataSample(recId, s));
    }
    CHECK(archive.FlushFlightData(recId));
    CHECK(archive.CloseFlightRecord(recId));
    CHECK(archive.SetRecordValid(recId));

    // Verify record 0 is undisturbed
    uint32_t count0 = 0u;
    CHECK(archive.GetFlightSampleCount(0u, count0));
    CHECK_EQ(count0, 250u);

    // Verify record 1 sample count
    uint32_t count1 = 0u;
    CHECK(archive.GetFlightSampleCount(recId, count1));
    CHECK_EQ(count1, kSampleCount);

    // Spot-check first and last sample of record 1
    FlightArchive::FlightSample buf[64];
    uint32_t got = 0u;
    CHECK(archive.ReadFlightDataRange(recId, 0u, buf, 64u, got));
    CHECK_EQ(got, kSampleCount);
    CheckSampleMatch(buf[0],              MakeSample(1000u),                    0u);
    CheckSampleMatch(buf[kSampleCount-1u], MakeSample(1000u + kSampleCount - 1u), kSampleCount - 1u);
}

// ---------------------------------------------------------------------------
// Test 3: RecoverFlightData on a truncated (not-closed) record
// ---------------------------------------------------------------------------

static void TestTruncatedRecordRecovery()
{
    printf("\n--- Test 3: truncated record recovery ---\n");

    RamFlashDriver freshFlash(kFlashSize);
    RocketArchive  archive(freshFlash, MakeConfig());
    CHECK(archive.Init());
    CHECK(archive.ScanArchive());

    const uint16_t recId = archive.GetNextAvailableArchiveRecord();
    CHECK(archive.PrepareRecord(recId));
    CHECK(archive.InitializeFlightRecord(recId));

    // Write enough samples to commit at least one full chunk, then stop
    // without closing (simulates a battery cut mid-flight).
    const uint32_t kChunkSamples =
        kChunkPayloadBytes / sizeof(FlightArchive::FlightSample);
    const uint32_t kWritten = kChunkSamples * 2u + 3u; // two full chunks + partial

    for (uint32_t i = 0; i < kWritten; ++i)
        archive.WriteFlightDataSample(recId, MakeSample(i));

    // Flush (commits the last partial chunk) but intentionally do NOT close.
    archive.FlushFlightData(recId);

    // Power cycle
    printf("  [simulating power cycle — rescanning]\n");
    RocketArchive archive2(freshFlash, MakeConfig());
    CHECK(archive2.Init());
    CHECK(archive2.ScanArchive());

    // Record should exist but not be valid (not closed)
    FlightArchive::Archive<FlightArchive::FlightSample,
                           FlightArchive::ExampleEventStats,
                           kChunkPayloadBytes>::RecordInfo info{};
    CHECK(archive2.GetRecordInfo(recId, info));
    CHECK(info.exists);
    CHECK(!info.closed);

    // RecoverFlightData should return the committed chunks
    std::vector<FlightArchive::FlightSample> recovered(kWritten + 10u);
    uint32_t recoveredCount = 0u;
    CHECK(archive2.RecoverFlightData(recId,
                                     recovered.data(),
                                     static_cast<uint32_t>(recovered.size()),
                                     recoveredCount));
    // Should have recovered at least the two full committed chunks
    printf("  Recovered %u samples from truncated record (wrote %u)\n",
           recoveredCount, kWritten);
    CHECK(recoveredCount >= kChunkSamples * 2u);

    // Spot-check a recovered sample
    if (recoveredCount > 0u)
        CheckSampleMatch(recovered[0], MakeSample(0u), 0u);
}

// ---------------------------------------------------------------------------
// Test 4: record wraps around (oldest record overwritten when all slots full)
// ---------------------------------------------------------------------------

static void TestRecordWrapAround()
{
    printf("\n--- Test 4: record wrap-around (all %u slots used) ---\n",
           kRecordCount);

    RamFlashDriver freshFlash(kFlashSize);
    RocketArchive  archive(freshFlash, MakeConfig());
    CHECK(archive.Init());
    CHECK(archive.ScanArchive());

    // Fill all record slots
    for (uint16_t r = 0u; r < kRecordCount; ++r)
    {
        uint16_t recId = archive.GetNextAvailableArchiveRecord();
        CHECK(archive.PrepareRecord(recId));
        CHECK(archive.InitializeFlightRecord(recId));
        // Write a tiny flight (1 sample) just to occupy the slot
        archive.WriteFlightDataSample(recId, MakeSample(r));
        archive.FlushFlightData(recId);
        archive.CloseFlightRecord(recId);
        archive.SetRecordValid(recId);
    }

    // One more flight should evict the oldest record and succeed
    uint16_t next = archive.GetNextAvailableArchiveRecord();
    CHECK(next != FlightArchive::INVALID_RECORD_ID);
    printf("  Next available record after wrap: %u\n", next);
    bool prepared = archive.PrepareRecord(next);
    CHECK(prepared);
    if (prepared)
    {
        archive.InitializeFlightRecord(next);
        archive.WriteFlightDataSample(next, MakeSample(999u));
        archive.FlushFlightData(next);
        archive.CloseFlightRecord(next);
        archive.SetRecordValid(next);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    printf("==========================================================\n");
    printf(" FlightArchive round-trip test\n");
    printf(" FlightSample layout: %zu bytes  (page: %u bytes)\n",
           sizeof(FlightArchive::FlightSample),
           RamFlashDriver::kPageSizeBytes);
    printf("==========================================================\n");

    // Tests 1 and 2 share one flash image so that slot-continuity is checked.
    RamFlashDriver sharedFlash(kFlashSize);
    TestSingleRecordRoundTrip(sharedFlash);
    TestSecondRecordIndependent(sharedFlash);

    // Tests 3 and 4 use fresh flash images.
    TestTruncatedRecordRecovery();
    TestRecordWrapAround();

    printf("\n==========================================================\n");
    printf(" Results: %d passed, %d failed\n", g_pass, g_fail);
    printf("==========================================================\n");
    return (g_fail == 0) ? 0 : 1;
}
