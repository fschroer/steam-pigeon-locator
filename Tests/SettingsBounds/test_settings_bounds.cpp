// ---------------------------------------------------------------------------
// Deployment settings bounds — SettingsBounds.hpp (ADR-0034, #50).
//
// Guards the one bounds table and the rules every boundary applies:
//   * IsValid — a config request failing it is rejected whole (decision 4);
//   * Sanitize — stored out-of-range fields reset to defaults at boot,
//     a primary/backup pair together (decision 5);
//   * StepLower / StepUpper — the console's stepping and the push rule
//     (decisions 1–2): raising the lower member of a pair to meet the upper
//     pushes the upper along until it reaches its own limit.
//
// The motivating record: Red Ryder Revisited flew with main primary 2,500 m and
// main backup 2,400 m (typed as feet), so both mains came due at apogee.
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstring>

#include "SettingsBounds.hpp"

static int g_pass = 0, g_fail = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (cond) ++g_pass;                                                    \
        else { ++g_fail; printf("  FAIL  %s  (line %d)\n", #cond, __LINE__); } \
    } while (0)

using namespace SettingsBounds;

static RocketPersistentSettings Defaults() {
    RocketPersistentSettings d{};
    std::strcpy(d.device_name, "Locator 0000ABCD");
    return d;
}

static void TestDefaultsAreValid() {
    printf("the shipped defaults pass their own bounds\n");
    CHECK(IsValid(Defaults()));
}

static void TestRedRyderIsRejected() {
    printf("Red Ryder's 2,500 / 2,400 m mains are rejected\n");
    RocketPersistentSettings s = Defaults();
    s.main_primary_deploy_altitude = 2500;
    s.main_backup_deploy_altitude  = 2400;
    CHECK(!IsValid(s));
}

static void TestTableEdges() {
    printf("the table's edges\n");
    RocketPersistentSettings s = Defaults();
    s.drogue_primary_deploy_delay = 20; s.drogue_backup_deploy_delay = 40;
    CHECK(IsValid(s));                                       // both at their limits
    s.drogue_primary_deploy_delay = 21;  CHECK(!IsValid(s)); // primary past 2.0 s
    s.drogue_primary_deploy_delay = 20; s.drogue_backup_deploy_delay = 41;
    CHECK(!IsValid(s));                                      // backup past 4.0 s
    s.drogue_backup_deploy_delay = 20;   CHECK(!IsValid(s)); // backup == primary
    s = Defaults();
    s.main_primary_deploy_altitude = 500; s.main_backup_deploy_altitude = 499;
    CHECK(IsValid(s));
    s.main_primary_deploy_altitude = 501; CHECK(!IsValid(s));
    s.main_primary_deploy_altitude = 499; CHECK(!IsValid(s)); // backup == primary
    s.main_primary_deploy_altitude = 1; s.main_backup_deploy_altitude = 0;
    CHECK(IsValid(s));                                        // the lowest legal pair
    s = Defaults(); s.lora_channel = 63; CHECK(IsValid(s));
    s.lora_channel = 64; CHECK(!IsValid(s));
    s = Defaults(); s.deployment_ch3_mode = static_cast<DeployMode>(5); CHECK(!IsValid(s));
    s = Defaults(); s.deployment_ch3_mode = DeployMode::Unused;         CHECK(IsValid(s));
    s = Defaults(); s.nose_axis = static_cast<NoseAxis>(4);             CHECK(!IsValid(s));
}

static void TestSanitizeResetsAPairTogether() {
    printf("boot repair resets an out-of-range pair to defaults, together\n");
    const RocketPersistentSettings d = Defaults();
    RocketPersistentSettings s = d;
    s.main_primary_deploy_altitude = 2500;
    s.main_backup_deploy_altitude  = 2400;
    s.drogue_backup_deploy_delay   = 9;          // Red Ryder's 0.9 s — legal, must survive
    CHECK(Sanitize(s, d));
    CHECK(s.main_primary_deploy_altitude == 130);
    CHECK(s.main_backup_deploy_altitude  == 100);
    CHECK(s.drogue_backup_deploy_delay   == 9);
    CHECK(IsValid(s));
    // Resetting only the bad member would leave this pair out of order:
    // primary 600 -> 130 with backup 300 is backup > primary.
    s = d; s.main_primary_deploy_altitude = 600; s.main_backup_deploy_altitude = 300;
    CHECK(Sanitize(s, d));
    CHECK(s.main_primary_deploy_altitude == 130 && s.main_backup_deploy_altitude == 100);
}

static void TestSanitizeLeavesValidSettingsAlone() {
    printf("boot repair leaves valid settings untouched and reports no change\n");
    const RocketPersistentSettings d = Defaults();
    RocketPersistentSettings s = d;
    s.main_primary_deploy_altitude = 300; s.main_backup_deploy_altitude = 250;
    s.drogue_primary_deploy_delay = 5;    s.drogue_backup_deploy_delay = 30;
    s.lora_channel = 44;
    RocketPersistentSettings before = s;
    CHECK(!Sanitize(s, d));
    CHECK(std::memcmp(&s, &before, sizeof s) == 0);
}

static void TestSanitizeRepairsTheName() {
    printf("an unterminated name is terminated, not rejected\n");
    RocketPersistentSettings s = Defaults();
    std::memset(s.device_name, 'A', device_name_length);
    CHECK(IsValid(s));                 // the name is not a deployment setting
    CHECK(TerminateName(s));
    CHECK(s.device_name[device_name_length - 1] == '\0');
}

static void TestDroguePush() {
    printf("drogue: raising the primary to meet the backup pushes the backup by 0.1 s\n");
    int primary = 18, backup = 19;
    StepLower(primary, backup, +1, kDroguePrimaryMax, kDrogueBackupMax);
    CHECK(primary == 19 && backup == 20);
    StepLower(primary, backup, +1, kDroguePrimaryMax, kDrogueBackupMax);
    CHECK(primary == 20 && backup == 21);
    StepLower(primary, backup, +1, kDroguePrimaryMax, kDrogueBackupMax);
    CHECK(primary == 20 && backup == 21);   // primary at its own 2.0 s limit
    // Lowering the backup to meet the primary pushes the primary DOWN by 0.1 s.
    StepUpper(primary, backup, -1, kDrogueBackupMax);
    CHECK(primary == 19 && backup == 20);
    for (int i = 0; i < 30; ++i) StepUpper(primary, backup, -1, kDrogueBackupMax);
    CHECK(primary == 0 && backup == 1);     // primary at 0: backup stops at 0.1 s
    StepUpper(primary, backup, -1, kDrogueBackupMax);
    CHECK(primary == 0 && backup == 1);
    // The backup stops at 4.0 s.
    for (int i = 0; i < 50; ++i) StepUpper(primary, backup, +1, kDrogueBackupMax);
    CHECK(backup == 40);
    // Lowering the primary never moves the backup; it stops at 0.
    for (int i = 0; i < 30; ++i) StepLower(primary, backup, -1, kDroguePrimaryMax, kDrogueBackupMax);
    CHECK(primary == 0 && backup == 40);
}

static void TestMainPush() {
    printf("main: raising the backup to meet the primary pushes the primary by 1 m, up to 500\n");
    int backup = 129, primary = 130;          // main: the BACKUP is the lower member
    StepLower(backup, primary, +1, kMainBackupStepMax, kMainPrimaryMax);
    CHECK(backup == 130 && primary == 131);
    backup = 498; primary = 499;
    StepLower(backup, primary, +1, kMainBackupStepMax, kMainPrimaryMax);
    CHECK(backup == 499 && primary == 500);
    StepLower(backup, primary, +1, kMainBackupStepMax, kMainPrimaryMax);
    CHECK(backup == 499 && primary == 500);   // primary at 500: backup stops at 499
    // Lowering the primary to meet the backup pushes the backup DOWN by 1 m.
    StepUpper(backup, primary, -1, kMainPrimaryMax);
    CHECK(backup == 498 && primary == 499);
    backup = 0; primary = 1;
    StepUpper(backup, primary, -1, kMainPrimaryMax);
    CHECK(backup == 0 && primary == 1);     // backup at 0: primary stops at 1 m
    backup = 498; primary = 499;
    // Every state the stepping can reach is valid.
    RocketPersistentSettings s = Defaults();
    s.main_primary_deploy_altitude = static_cast<uint16_t>(primary);
    s.main_backup_deploy_altitude  = static_cast<uint16_t>(backup);
    CHECK(IsValid(s));
}

static void TestSteppingNeverLeavesTheTable() {
    printf("10,000 random steps on both pairs never produce an invalid setting\n");
    unsigned seed = 50;
    auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return (seed >> 16) & 0x7FFF; };
    int dp = 0, db = 20, mb = 100, mp = 130;
    int bad = 0;
    for (int i = 0; i < 10000; ++i) {
        const int delta = (rnd() & 1) ? 1 : -1;
        switch (rnd() % 4) {
        case 0: StepLower(dp, db, delta, kDroguePrimaryMax, kDrogueBackupMax); break;
        case 1: StepUpper(dp, db, delta, kDrogueBackupMax); break;
        case 2: StepLower(mb, mp, delta, kMainBackupStepMax, kMainPrimaryMax); break;
        case 3: StepUpper(mb, mp, delta, kMainPrimaryMax); break;
        }
        if (!DroguePairValid(dp, db) || !MainPairValid(mp, mb)) ++bad;
    }
    CHECK(bad == 0);
}

int main() {
    TestDefaultsAreValid();
    TestRedRyderIsRejected();
    TestTableEdges();
    TestSanitizeResetsAPairTogether();
    TestSanitizeLeavesValidSettingsAlone();
    TestSanitizeRepairsTheName();
    TestDroguePush();
    TestMainPush();
    TestSteppingNeverLeavesTheTable();
    // "Results:" is what .githooks/pre-commit greps for.
    printf("\n Results: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
