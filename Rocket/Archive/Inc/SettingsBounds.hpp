#pragma once

// SettingsBounds — the one bounds table for the locator's deployment settings,
// and the rules that keep each primary/backup pair in order (ADR-0034, #50).
//
// Red Ryder Revisited flew with main primary 2,500 m / backup 2,400 m (typed as
// feet): both mains came due at apogee.  The app's bounds only applied to its
// arrow buttons, the firmware applied none, and the console's disagreed with
// both.  This header is now the firmware's single definition, used by:
//
//   * Communication — a LocatorCfgChgRequest that fails IsValid() is rejected
//     whole (ADR-0034 decision 4);
//   * Archive::Init — Sanitize() resets stored out-of-range fields to defaults
//     at boot (decision 5);
//   * the USB console — its [ / ] stepping uses StepLower / StepUpper, so it
//     obeys the same limits and the push rule (decisions 1–2).
//
// The Android app's LocatorSettingsBounds.kt carries the same table; change
// both together.  HAL-free and header-only so Tests/SettingsBounds compiles it.

#include <cstdint>
#include <cstring>

#include "RocketSettings.hpp"

// Flash stopgap (#50): the flight build compiles at -O0 and the 256 KB flash
// had under 800 B free, so this header alone (~1.1 KB at -O0) overflowed it.
// Only the functions between push_options and pop_options are size-optimized;
// every other translation unit is unaffected.  Remove when the build moves off
// -O0 (#57).
#pragma GCC push_options
#pragma GCC optimize("Os")

namespace SettingsBounds {

// Drogue delays, in tenths of a second.
constexpr int kDroguePrimaryMax = 20;   // 2.0 s
constexpr int kDrogueBackupMax  = 40;   // 4.0 s
// Main altitudes, in meters.  The main pair is the other way up: its LOWER
// member is the backup (it fires lower, later).
constexpr int kMainPrimaryMax   = 500;
constexpr int kLoraChannelMax   = 63;

inline bool ModeValid(DeployMode m) {
    switch (m) {
    case DeployMode::DroguePrimary: case DeployMode::DrogueBackup:
    case DeployMode::MainPrimary:   case DeployMode::MainBackup:
    case DeployMode::Unused:
        return true;
    }
    return false;
}

inline bool AxisValid(NoseAxis a) {
    switch (a) {
    case NoseAxis::Auto: case NoseAxis::X: case NoseAxis::Y: case NoseAxis::Z:
        return true;
    }
    return false;
}

inline bool DroguePairValid(int primary, int backup) {
    return primary >= 0 && primary <= kDroguePrimaryMax
        && backup <= kDrogueBackupMax && backup > primary;
}

inline bool MainPairValid(int primary, int backup) {
    return backup >= 0 && primary <= kMainPrimaryMax && primary > backup;
}

/** Every deployment-relevant field in range and in order.  The device name is
 *  not checked here: a missing terminator is repaired, not grounds to reject. */
inline bool IsValid(const RocketPersistentSettings& s) {
    return DroguePairValid(s.drogue_primary_deploy_delay, s.drogue_backup_deploy_delay)
        && MainPairValid(s.main_primary_deploy_altitude, s.main_backup_deploy_altitude)
        && s.lora_channel <= kLoraChannelMax
        && ModeValid(s.deployment_ch1_mode) && ModeValid(s.deployment_ch2_mode)
        && ModeValid(s.deployment_ch3_mode) && ModeValid(s.deployment_ch4_mode)
        && AxisValid(s.nose_axis);
}

/** Force the device name's terminator.  Returns true if it had to. */
inline bool TerminateName(RocketPersistentSettings& s) {
    if (s.device_name[device_name_length - 1] == '\0') return false;
    s.device_name[device_name_length - 1] = '\0';
    return true;
}

/**
 * Boot-time repair (ADR-0034 decision 5): reset each out-of-range field to its
 * default — a primary/backup pair together, since resetting one could leave it
 * out of order with the other.  Returns true if anything changed, so the
 * caller can save the repaired settings.
 */
inline bool Sanitize(RocketPersistentSettings& s, const RocketPersistentSettings& d) {
    bool changed = false;
    if (!DroguePairValid(s.drogue_primary_deploy_delay, s.drogue_backup_deploy_delay)) {
        s.drogue_primary_deploy_delay = d.drogue_primary_deploy_delay;
        s.drogue_backup_deploy_delay  = d.drogue_backup_deploy_delay;
        changed = true;
    }
    if (!MainPairValid(s.main_primary_deploy_altitude, s.main_backup_deploy_altitude)) {
        s.main_primary_deploy_altitude = d.main_primary_deploy_altitude;
        s.main_backup_deploy_altitude  = d.main_backup_deploy_altitude;
        changed = true;
    }
    if (s.lora_channel > kLoraChannelMax) { s.lora_channel = d.lora_channel; changed = true; }
    if (!ModeValid(s.deployment_ch1_mode)) { s.deployment_ch1_mode = d.deployment_ch1_mode; changed = true; }
    if (!ModeValid(s.deployment_ch2_mode)) { s.deployment_ch2_mode = d.deployment_ch2_mode; changed = true; }
    if (!ModeValid(s.deployment_ch3_mode)) { s.deployment_ch3_mode = d.deployment_ch3_mode; changed = true; }
    if (!ModeValid(s.deployment_ch4_mode)) { s.deployment_ch4_mode = d.deployment_ch4_mode; changed = true; }
    if (!AxisValid(s.nose_axis)) { s.nose_axis = d.nose_axis; changed = true; }
    if (TerminateName(s)) changed = true;
    return changed;
}

// ── The push rule (ADR-0034 decision 2), on a (lower, upper) pair ──────────
//
//   drogue: lower = primary delay (max 20), upper = backup delay (max 40)
//   main:   lower = backup altitude,         upper = primary altitude (max 500)
//
// The push rule works both ways (fschroer, 2026-10-01).  Raising the lower
// member to meet the upper one pushes the upper one up a step, until the
// upper reaches its own limit; then the lower stops one step below it.
// Lowering the upper member to meet the lower one pushes the lower one down a
// step, until the lower reaches 0; then the upper stops one step above it.

/** Step the lower member by `delta` (±1), within [0, lowerMax]. */
inline void StepLower(int& lower, int& upper, int delta, int lowerMax, int upperMax) {
    int next = lower + delta;
    if (next < 0) next = 0;
    if (next > lowerMax) next = lowerMax;
    if (next >= upper) {
        if (next + 1 <= upperMax) upper = next + 1;   // push
        else next = upper - 1;                        // upper at its limit
    }
    lower = next;
}

/** Step the upper member by `delta` (±1), within [1, upperMax]. */
inline void StepUpper(int& lower, int& upper, int delta, int upperMax) {
    int next = upper + delta;
    if (next > upperMax) next = upperMax;
    if (next < 1) next = 1;
    if (next <= lower) {
        if (next - 1 >= 0) lower = next - 1;          // push down
        else next = lower + 1;                        // lower at 0
    }
    upper = next;
}

// The main backup has no limit of its own beyond staying below the primary.
constexpr int kMainBackupStepMax = kMainPrimaryMax - 1;

}  // namespace SettingsBounds

#pragma GCC pop_options
