// ---------------------------------------------------------------------------
// Sensor faults — SensorFaults.hpp (ADR-0038).
//
// Guards the rule that replaced the restart loop:
//   * Failed — Error, Off and Initializing are failures; Ok, Warning and Stale
//     are not (a transient read failure or a GPS with no fresh fix is a sensor
//     that is up and may recover);
//   * ArmingAllowed — needs the IMU and the barometer, never the GPS;
//   * FailedMask — one bit per failed sensor.
//
// The Android app mirrors this rule in SensorFaults.kt and its test pins the
// same table.  A row changed here is a row to change there.
//
// The motivating flight: a ballistic return damaged one sensor, and the locator
// restarted every few seconds from then on, so its record could not be read.
// ---------------------------------------------------------------------------
#include <cstdio>

#include "SensorFaults.hpp"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

using namespace SensorFaults;

static constexpr SensorHealth kAll[] = {
    SensorHealth::Off, SensorHealth::Initializing, SensorHealth::Ok,
    SensorHealth::Warning, SensorHealth::Error, SensorHealth::Stale,
};

static void TestFailedTable() {
    printf("which health values are a failed sensor\n");
    CHECK(Failed(SensorHealth::Error));
    CHECK(Failed(SensorHealth::Off));
    CHECK(Failed(SensorHealth::Initializing));
    CHECK(!Failed(SensorHealth::Ok));
    CHECK(!Failed(SensorHealth::Warning));   // one bad read is not a dead sensor
    CHECK(!Failed(SensorHealth::Stale));     // a GPS waiting on a fix is up
}

static void TestWireValuesUnchanged() {
    printf("the health enum keeps its wire values (the app decodes these bytes)\n");
    CHECK(static_cast<int>(SensorHealth::Off)          == 0);
    CHECK(static_cast<int>(SensorHealth::Initializing) == 1);
    CHECK(static_cast<int>(SensorHealth::Ok)           == 2);
    CHECK(static_cast<int>(SensorHealth::Warning)      == 3);
    CHECK(static_cast<int>(SensorHealth::Error)        == 4);
    CHECK(static_cast<int>(SensorHealth::Stale)        == 5);
}

static void TestArmingNeedsImuAndBaro() {
    printf("arming needs the IMU and the barometer\n");
    CHECK(ArmingAllowed(SensorHealth::Ok, SensorHealth::Ok));
    CHECK(!ArmingAllowed(SensorHealth::Error, SensorHealth::Ok));
    CHECK(!ArmingAllowed(SensorHealth::Ok, SensorHealth::Error));
    CHECK(!ArmingAllowed(SensorHealth::Error, SensorHealth::Error));
    // A transient warning on either does not hold an arm: it would make arming
    // flicker with every bus hiccup on the pad.
    CHECK(ArmingAllowed(SensorHealth::Warning, SensorHealth::Warning));
}

static void TestArmingIsExactlyTheTwoSensorsUp() {
    printf("arming over every IMU x baro pair\n");
    for (SensorHealth imu : kAll)
        for (SensorHealth baro : kAll)
            CHECK(ArmingAllowed(imu, baro) == (!Failed(imu) && !Failed(baro)));
}

static void TestMask() {
    printf("one bit per failed sensor\n");
    CHECK(FailedMask(SensorHealth::Ok, SensorHealth::Ok, SensorHealth::Ok) == 0);
    CHECK(FailedMask(SensorHealth::Error, SensorHealth::Ok, SensorHealth::Ok) == kImu);
    CHECK(FailedMask(SensorHealth::Ok, SensorHealth::Error, SensorHealth::Ok) == kBaro);
    CHECK(FailedMask(SensorHealth::Ok, SensorHealth::Ok, SensorHealth::Error) == kGps);
    CHECK(FailedMask(SensorHealth::Error, SensorHealth::Error, SensorHealth::Error)
          == (kImu | kBaro | kGps));
    // A stale GPS is not a failed one — the status LED would otherwise flash
    // yellow on every pad until the first fix.
    CHECK(FailedMask(SensorHealth::Ok, SensorHealth::Ok, SensorHealth::Stale) == 0);
}

static_assert(Failed(SensorHealth::Error), "usable in constant expressions");
static_assert(!ArmingAllowed(SensorHealth::Ok, SensorHealth::Error), "usable in constant expressions");

int main() {
    printf("SensorFaults (ADR-0038)\n");
    TestFailedTable();
    TestWireValuesUnchanged();
    TestArmingNeedsImuAndBaro();
    TestArmingIsExactlyTheTwoSensorsUp();
    TestMask();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
